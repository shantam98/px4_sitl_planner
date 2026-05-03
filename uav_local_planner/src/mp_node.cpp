// mp_node.cpp
// ROS 2 node wrapping the Motion Primitive Library local planner.
// Drop-in replacement for vfh3d_node — identical I/O topics.
//
// Subscribes:
//   /drone/tof_merged/points   (sensor_msgs/PointCloud2)
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
#include <deque>
#include <mutex>
#include <cmath>

// ── Bounding-box stall detector ───────────────────────────────────────────
// Tracks XY positions over a sliding time window. If the diagonal of the
// bounding box stays below stall_radius_xy, the drone is considered stalled.
//
// Uses the bounding box (not centroid distance) so that VIO drift — which
// slowly walks the centroid but keeps the box compact — does not suppress
// detection. A drone orbiting in a tight circle will have a box diagonal of
// ~2× orbit radius, exposing it even when centroid displacement is small.
struct StallDetector {
  struct Sample { rclcpp::Time t; double x, y; };

  double window_sec    = 3.0;   // sliding window length
  double radius_xy     = 0.8;   // bounding-box diagonal threshold (metres)
  int    min_samples   = 15;    // require ≥ this many samples before firing
  bool   stalled       = false;

  std::deque<Sample> buf;

  void reset() { buf.clear(); stalled = false; }

  // Call once per update cycle with the current XY position and wall time.
  void update(double x, double y, rclcpp::Time now)
  {
    buf.push_back({now, x, y});

    // Prune entries older than the window
    while (!buf.empty() &&
           (now - buf.front().t).seconds() > window_sec)
      buf.pop_front();

    if (static_cast<int>(buf.size()) < min_samples) {
      stalled = false;
      return;
    }

    // Bounding box in XY
    double xmin = buf.front().x, xmax = xmin;
    double ymin = buf.front().y, ymax = ymin;
    for (const auto& s : buf) {
      xmin = std::min(xmin, s.x); xmax = std::max(xmax, s.x);
      ymin = std::min(ymin, s.y); ymax = std::max(ymax, s.y);
    }

    const double diag = std::sqrt(
        (xmax - xmin) * (xmax - xmin) +
        (ymax - ymin) * (ymax - ymin));

    stalled = (diag < radius_xy);
  }
};

// ─────────────────────────────────────────────────────────────────────────
class MPNode : public rclcpp::Node
{
public:
  MPNode() : Node("mp_node")
  {
    // ── Parameters ──────────────────────────────────────────────────────
    uav_local_planner::MPConfig cfg;

    // Arc primitives
    cfg.num_az_horizontal = declare_parameter("num_az_horizontal", 18);
    cfg.num_curvatures    = declare_parameter("num_curvatures",     5);
    cfg.max_curvature     = declare_parameter("max_curvature",      0.4);
    cfg.arc_length        = declare_parameter("arc_length",         2.0);

    // Pitched layers
    cfg.elevation_angles_deg = declare_parameter(
        "elevation_angles_deg", std::vector<double>{30.0, 15.0, -15.0, -30.0});
    cfg.num_az_pitched    = declare_parameter("num_az_pitched", 18);

    // Collision
    cfg.collision_radius  = declare_parameter("collision_radius", 0.45);
    cfg.min_clearance     = declare_parameter("min_clearance",    0.6);

    // Speed
    cfg.max_speed         = declare_parameter("max_speed",  2.5);
    cfg.min_speed         = declare_parameter("min_speed",  0.2);
    cfg.max_vz            = declare_parameter("max_vz",     1.0);
    cfg.alt_kp            = declare_parameter("alt_kp",     0.8);

    // Scoring
    cfg.w_goal            = declare_parameter("w_goal", 3.0);
    cfg.w_prev            = declare_parameter("w_prev", 2.0);

    // History buffer
    cfg.history_capacity  = declare_parameter("history_capacity",  2048);
    cfg.history_subsample = declare_parameter("history_subsample", 4);

    // Stall detection
    stall_.window_sec  = declare_parameter("stall_window_sec",  3.0);
    stall_.radius_xy   = declare_parameter("stall_radius_xy",   0.8);
    stall_.min_samples = declare_parameter("stall_min_samples", 15);

    double rate_hz = declare_parameter("update_rate_hz", 20.0);

    planner_ = std::make_unique<uav_local_planner::MotionPrimitives>(cfg);

    RCLCPP_INFO(get_logger(),
        "MPNode: %d horiz arcs (%d az × %d κ) + %d pitched segments, %.0f Hz | "
        "stall: %.1fs window / %.2fm bbox",
        planner_->numHorizPrims(),
        cfg.num_az_horizontal, cfg.num_curvatures,
        planner_->numPitchedPrims(),
        rate_hz,
        stall_.window_sec, stall_.radius_xy);

    // ── Subscribers ──────────────────────────────────────────────────────
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
          auto& q = msg->pose.pose.orientation;
          Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
          yaw_ = std::atan2(
              2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
              1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
          have_odom_ = true;
        });

    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/uav/current_waypoint", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          waypoint_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
          have_waypoint_ = true;
          planner_->reset();
          stall_.reset();   // new waypoint clears stall state
        });

    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          have_waypoint_ = false;
          if (msg->data) { planner_->reset(); stall_.reset(); }
        });

    // ── Publishers ───────────────────────────────────────────────────────
    cmd_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>("/uav/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/uav/vfh_status", 10);

    // ── Timer ────────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&MPNode::update, this));
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

    // ── Stall check ──────────────────────────────────────────────────────
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

    stall_.update(pos.x(), pos.y(), get_clock()->now());

    if (stall_.stalled) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "STALLED: XY bounding box < %.2fm over %.1fs — hovering. "
          "Send a new waypoint to resume.",
          stall_.radius_xy, stall_.window_sec);
      status.data = "STALLED";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp    = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);   // zero velocity
      return;
    }

    // ── Cloud check ──────────────────────────────────────────────────────
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    {
      std::lock_guard<std::mutex> lk(cloud_mutex_);
      cloud = cloud_;
    }
    if (!cloud) return;

    // ── Run planner ──────────────────────────────────────────────────────
    auto result = planner_->update(*cloud, pos, yaw, wp);

    // ── Publish cmd_vel ──────────────────────────────────────────────────
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

    // ── Publish status ───────────────────────────────────────────────────
    status.data = result.estop            ? "ESTOP"    :
                  result.obstacle_detected ? "AVOIDING" : "NOMINAL";
    status_pub_->publish(status);
  }

  std::unique_ptr<uav_local_planner::MotionPrimitives> planner_;
  StallDetector stall_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
  Eigen::Vector3d pos_{0, 0, 0}, waypoint_{0, 0, 0};
  double yaw_{0.0};
  bool have_odom_{false}, have_waypoint_{false};

  std::mutex cloud_mutex_, odom_mutex_, wp_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
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
