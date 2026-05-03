// mp_node.cpp
// ROS 2 node wrapping the Motion Primitive Library local planner.
// Drop-in replacement for vfh3d_node — identical I/O topics.
//
// Subscribes:
//   /drone/tof_merged/points   (sensor_msgs/PointCloud2)  ← raw ToF cloud
//   /drone/odom                (nav_msgs/Odometry)
//   /uav/current_waypoint      (geometry_msgs/PointStamped)
//   /uav/mission_complete      (std_msgs/Bool)
//
// Publishes:
//   /uav/cmd_vel               (geometry_msgs/TwistStamped)
//   /uav/vfh_status            (std_msgs/String)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <uav_local_planner/motion_primitives.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <cmath>

class MPNode : public rclcpp::Node
{
public:
  MPNode() : Node("mp_node")
  {
    // ── Parameters ─────────────────────────────────────────────────────
    uav_local_planner::MPConfig cfg;
    cfg.num_primitives   = declare_parameter("num_primitives",   72);
    cfg.primitive_length = declare_parameter("primitive_length", 2.0);
    cfg.collision_radius = declare_parameter("collision_radius", 0.45);
    cfg.max_speed        = declare_parameter("max_speed",        2.5);
    cfg.min_speed        = declare_parameter("min_speed",        0.2);
    cfg.max_vz           = declare_parameter("max_vz",           1.0);
    cfg.min_clearance    = declare_parameter("min_clearance",    0.6);
    cfg.w_goal           = declare_parameter("w_goal",           3.0);
    cfg.w_prev           = declare_parameter("w_prev",           2.0);
    cfg.alt_kp           = declare_parameter("alt_kp",           0.8);
    double rate_hz       = declare_parameter("update_rate_hz",   20.0);

    planner_ = std::make_unique<uav_local_planner::MotionPrimitives>(cfg);

    // ── Subscribers ────────────────────────────────────────────────────
    // Match the Reliable QoS used by cloud_merge_node publisher.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/drone/tof_merged/points",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
          pcl::fromROSMsg(*msg, *cloud);
          std::lock_guard<std::mutex> lk(cloud_mutex_);
          cloud_ = cloud;
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(odom_mutex_);
          pos_ = Eigen::Vector3d(
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          // Extract yaw from quaternion
          auto& q = msg->pose.pose.orientation;
          Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
          Eigen::Matrix3d R = quat.toRotationMatrix();
          yaw_ = std::atan2(R(1, 0), R(0, 0));
          have_odom_ = true;
        });

    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/uav/current_waypoint", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          waypoint_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
          have_waypoint_ = true;
          planner_->reset();
        });

    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          have_waypoint_ = false;
          if (msg->data) planner_->reset();
        });

    // ── Publishers ─────────────────────────────────────────────────────
    cmd_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>("/uav/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/uav/vfh_status", 10);

    // ── Timer ──────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&MPNode::update, this));

    RCLCPP_INFO(get_logger(), "MPNode (Motion Primitive Library) ready at %.0f Hz", rate_hz);
  }

private:
  void update()
  {
    if (!have_odom_) return;

    std_msgs::msg::String status;

    if (!have_waypoint_) {
      status.data = "IDLE";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp    = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      return;
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    {
      std::lock_guard<std::mutex> lk(cloud_mutex_);
      cloud = cloud_;
    }
    if (!cloud) return;

    Eigen::Vector3d pos, wp;
    double yaw;
    {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      pos = pos_; yaw = yaw_;
    }
    {
      std::lock_guard<std::mutex> lk(wp_mutex_);
      wp = waypoint_;
    }

    auto result = planner_->update(*cloud, pos, yaw, wp);

    // ── Publish cmd_vel ────────────────────────────────────────────────
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = get_clock()->now();
    cmd.header.frame_id = "map";

    if (result.estop) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "E-STOP: obstacle within %.2fm", result.closest_obstacle_dist);
    } else {
      cmd.twist.linear.x = result.velocity.x();
      cmd.twist.linear.y = result.velocity.y();
      cmd.twist.linear.z = result.velocity.z();
    }
    cmd_pub_->publish(cmd);

    // ── Publish status ─────────────────────────────────────────────────
    status.data = result.estop            ? "ESTOP"   :
                  result.obstacle_detected ? "AVOIDING" : "NOMINAL";
    status_pub_->publish(status);
  }

  std::unique_ptr<uav_local_planner::MotionPrimitives> planner_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
  Eigen::Vector3d pos_{0, 0, 0}, waypoint_{0, 0, 0};
  double yaw_{0.0};
  bool have_odom_{false}, have_waypoint_{false};

  std::mutex cloud_mutex_, odom_mutex_, wp_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              mission_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr               status_pub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPNode>());
  rclcpp::shutdown();
}
