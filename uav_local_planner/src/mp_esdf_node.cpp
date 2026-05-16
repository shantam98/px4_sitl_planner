// mp_esdf_node.cpp
// ESDF variant of mp_node — same state machine, same I/O, but obstacle
// scoring uses nvblox's static_esdf_pointcloud instead of the raw cloud
// history. Built as a separate executable so we can A/B against mp_node
// in the benchmark without touching the baseline binary.
//
// Subscribes:
//   /nvblox_node/static_esdf_pointcloud  (sensor_msgs/PointCloud2 with intensity)
//   /drone/odom                          (nav_msgs/Odometry)
//   /uav/current_waypoint                (geometry_msgs/PointStamped)
//   /uav/mission_complete                (std_msgs/Bool)
//
// Publishes:
//   /uav/cmd_vel                         (geometry_msgs/TwistStamped)
//   /uav/vfh_status                      (std_msgs/String)
//   /uav/mp_diag                         (std_msgs/Float64MultiArray)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <uav_local_planner/motion_primitives.hpp>

#include <Eigen/Dense>
#include <mutex>
#include <cmath>

namespace {
double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

// ── Detectors (identical to mp_node) ───────────────────────────────────────
struct OrbitDetector {
  double yaw_threshold  = 1.5 * M_PI;
  double progress_min   = 0.3;
  double ignore_radius  = 1.2;
  double yaw_accumulator{0.0};
  double min_dist{1e9};
  double prev_yaw_{0.0};
  bool   prev_valid_{false};
  bool   orbiting{false};

  void reset() {
    yaw_accumulator = 0.0; min_dist = 1e9; prev_valid_ = false; orbiting = false;
  }
  void update(double dist_to_goal, double yaw) {
    orbiting = false;
    if (dist_to_goal < ignore_radius) { reset(); return; }
    if (!prev_valid_) { prev_yaw_ = yaw; prev_valid_ = true; return; }
    double dyaw = std::abs(normalizeAngle(yaw - prev_yaw_));
    prev_yaw_ = yaw;
    yaw_accumulator += dyaw;
    double progress = min_dist - dist_to_goal;
    if (dist_to_goal < min_dist) min_dist = dist_to_goal;
    if (progress > progress_min) yaw_accumulator = 0.0;
    orbiting = (yaw_accumulator > yaw_threshold);
  }
};

struct StallDetector {
  double no_improve_window = 20.0;
  double progress_min      = 0.15;
  double ignore_radius     = 0.25;
  bool   stalled           = false;
  double bestDist() const { return best_dist_; }
  double sinceImprove() const { return since_improve_; }
  void reset() { best_dist_ = 1e9; last_improve_t_ = -1.0; since_improve_ = 0.0; stalled = false; }
  void update(double dist_to_goal, rclcpp::Time now) {
    if (dist_to_goal < ignore_radius) { reset(); return; }
    const double t = now.seconds();
    if (last_improve_t_ < 0.0) {
      best_dist_ = dist_to_goal; last_improve_t_ = t; stalled = false; return;
    }
    if (dist_to_goal < best_dist_ - progress_min) {
      best_dist_ = dist_to_goal; last_improve_t_ = t;
    }
    since_improve_ = t - last_improve_t_;
    stalled = (since_improve_ > no_improve_window);
  }
 private:
  double best_dist_{1e9};
  double last_improve_t_{-1.0};
  double since_improve_{0.0};
};

// ─────────────────────────────────────────────────────────────────────────
class MPEsdfNode : public rclcpp::Node
{
public:
  MPEsdfNode() : Node("mp_esdf_node")
  {
    uav_local_planner::MPConfig cfg;

    cfg.num_az_horizontal = declare_parameter("num_az_horizontal", 18);
    cfg.num_curvatures    = declare_parameter("num_curvatures",     5);
    cfg.max_curvature     = declare_parameter("max_curvature",      0.4);
    cfg.arc_length        = declare_parameter("arc_length",         2.0);

    cfg.elevation_angles_deg = declare_parameter(
        "elevation_angles_deg", std::vector<double>{30.0, 15.0, -15.0, -30.0});
    cfg.num_az_pitched    = declare_parameter("num_az_pitched", 18);

    cfg.collision_radius  = declare_parameter("collision_radius", 0.75);
    cfg.min_clearance     = declare_parameter("min_clearance",    1.0);

    cfg.max_speed         = declare_parameter("max_speed",  2.5);
    cfg.min_speed         = declare_parameter("min_speed",  0.2);
    cfg.max_vz            = declare_parameter("max_vz",     1.0);
    cfg.alt_kp            = declare_parameter("alt_kp",     0.8);

    cfg.w_goal            = declare_parameter("w_goal", 3.0);
    cfg.w_prev            = declare_parameter("w_prev", 2.0);
    cfg.w_pos             = declare_parameter("w_pos",  2.0);

    cfg.obstacle_hysteresis_factor = declare_parameter("obstacle_hysteresis_factor", 1.3);
    cfg.bypass_min_hold_cycles     = declare_parameter("bypass_min_hold_cycles",     60);

    // History buffer is unused in the ESDF path but MPConfig requires the
    // fields; leave defaults.
    cfg.history_capacity  = declare_parameter("history_capacity",  2048);
    cfg.history_subsample = declare_parameter("history_subsample", 4);

    stall_.no_improve_window = declare_parameter("stall_no_improve_window", 20.0);
    stall_.progress_min      = declare_parameter("stall_progress_min",       0.15);
    stall_.ignore_radius     = declare_parameter("stall_ignore_radius",      0.25);

    orbit_.yaw_threshold  = declare_parameter("orbit_yaw_threshold",  1.5 * M_PI);
    orbit_.progress_min   = declare_parameter("orbit_progress_min",   0.3);
    orbit_.ignore_radius  = declare_parameter("orbit_ignore_radius",  0.3);

    rate_hz_ = declare_parameter("update_rate_hz", 20.0);
    max_accel_ = declare_parameter("max_accel", 0.75);

    cfg.pessimistic_window = declare_parameter("pessimistic_window", 10);

    double recovery_duration_sec = declare_parameter("recovery_duration_sec", 2.0);
    cfg.recovery_max_speed = declare_parameter("recovery_max_speed", 0.5);
    cfg.recovery_duration_cycles = static_cast<int>(recovery_duration_sec * rate_hz_);

    // ESDF-specific parameters
    esdf_voxel_size_ = declare_parameter("esdf_voxel_size", 0.05);
    const std::string esdf_topic = declare_parameter(
        "esdf_topic", std::string("/nvblox_node/static_esdf_pointcloud"));

    planner_ = std::make_unique<uav_local_planner::MotionPrimitives>(cfg);

    RCLCPP_INFO(get_logger(),
        "MPEsdfNode: %d horiz arcs + %d pitched, %.0f Hz, ESDF voxel %.3f m, topic '%s'",
        planner_->numHorizPrims(), planner_->numPitchedPrims(),
        rate_hz_, esdf_voxel_size_, esdf_topic.c_str());

    // ── Subscribers ──────────────────────────────────────────────────────
    // ESDF cloud — best_effort matches nvblox's typical QoS for sensor-like topics.
    esdf_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        esdf_topic,
        rclcpp::QoS(5).best_effort().durability_volatile(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
          pcl::fromROSMsg(*msg, *cloud);
          {
            std::lock_guard<std::mutex> lk(planner_mutex_);
            planner_->updateEsdf(*cloud, esdf_voxel_size_);
          }
          have_esdf_ = true;
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
          {
            std::lock_guard<std::mutex> lk2(planner_mutex_);
            planner_->reset();
          }
          stall_.reset(); orbit_.reset();
        });

    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            std::lock_guard<std::mutex> lk(wp_mutex_);
            have_waypoint_ = false;
            {
              std::lock_guard<std::mutex> lk2(planner_mutex_);
              planner_->reset();
            }
            stall_.reset(); orbit_.reset();
          }
        });

    cmd_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>("/uav/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/uav/vfh_status", 10);
    diag_pub_   = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/mp_diag", 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        std::bind(&MPEsdfNode::tick, this));
  }

private:
  void tick()
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

    Eigen::Vector3d pos, wp;
    double yaw;
    { std::lock_guard<std::mutex> lk(odom_mutex_); pos = pos_; yaw = yaw_; }
    { std::lock_guard<std::mutex> lk(wp_mutex_); wp = waypoint_; }

    const double dist_to_goal = (pos.head<2>() - wp.head<2>()).norm();
    stall_.update(dist_to_goal, get_clock()->now());

    if (stall_.stalled) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "STALLED: best %.2fm, no %.2fm improvement for %.0fs",
          stall_.bestDist(), stall_.progress_min, stall_.sinceImprove());
      status.data = "STALLED";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      publishDiag(dist_to_goal, yaw, 0.0, 1.0, 0.0, 0.0, -1.0, false, false);
      return;
    }

    orbit_.update(dist_to_goal, yaw);
    if (orbit_.orbiting) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "ORBIT DETECTED: %.0f° accumulated yaw", orbit_.yaw_accumulator * 180.0 / M_PI);
      status.data = "ORBITING";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      publishDiag(dist_to_goal, yaw, 1.0, 0.0, 0.0, 0.0, -1.0, false, false);
      return;
    }

    if (!have_esdf_) {
      // No ESDF data yet — keep silent (hover) without ESTOP.
      status.data = "WAIT_ESDF";
      status_pub_->publish(status);
      return;
    }

    // ── Run ESDF-based planner ───────────────────────────────────────────
    uav_local_planner::MPResult result;
    {
      std::lock_guard<std::mutex> lk(planner_mutex_);
      result = planner_->updateWithEsdf(pos, yaw, wp);
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = get_clock()->now();
    cmd.header.frame_id = "map";

    if (result.estop) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "E-STOP (ESDF): obstacle within %.2fm", result.closest_obstacle_dist);
      prev_cmd_ = Eigen::Vector3d::Zero();
    } else {
      const double max_dv = max_accel_ / rate_hz_;
      Eigen::Vector3d target(result.velocity.x(), result.velocity.y(), result.velocity.z());
      cmd.twist.linear.x = std::clamp(target.x(), prev_cmd_.x() - max_dv, prev_cmd_.x() + max_dv);
      cmd.twist.linear.y = std::clamp(target.y(), prev_cmd_.y() - max_dv, prev_cmd_.y() + max_dv);
      cmd.twist.linear.z = std::clamp(target.z(), prev_cmd_.z() - max_dv, prev_cmd_.z() + max_dv);
      prev_cmd_ = Eigen::Vector3d(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);
    }
    cmd_pub_->publish(cmd);

    auto bs = planner_->bypassState();
    if (result.estop) {
      status.data = "ESTOP";
    } else if (planner_->inRecovery()) {
      status.data = "RECOVERING";
    } else if (result.obstacle_detected) {
      status.data = (bs == uav_local_planner::BypassState::LEFT)  ? "AVOIDING_L" :
                    (bs == uav_local_planner::BypassState::RIGHT) ? "AVOIDING_R" : "AVOIDING";
    } else {
      status.data = "NOMINAL";
    }
    status_pub_->publish(status);

    const double bs_val = (bs == uav_local_planner::BypassState::LEFT)  ? 1.0 :
                          (bs == uav_local_planner::BypassState::RIGHT) ? 2.0 : 0.0;
    publishDiag(dist_to_goal, yaw, 0.0, 0.0, bs_val,
                result.closest_obstacle_dist,
                static_cast<double>(result.best_primitive_idx),
                result.estop, result.obstacle_detected);
  }

  // Diag layout (matches mp_node, plus 2 extras at the end):
  //   [dist_to_goal, yaw, yaw_accum, orbiting, stalled, bypass_state,
  //    stall_best_dist, stall_since_improve, closest_obstacle,
  //    best_prim_idx, estop, obstacle_detected,
  //    esdf_mode (1.0), esdf_voxel_count]
  void publishDiag(double dist_to_goal, double yaw,
                   double orbiting, double stalled, double bs_val,
                   double closest_obs, double best_idx,
                   bool estop, bool obstacle_det)
  {
    std_msgs::msg::Float64MultiArray diag;
    diag.data = {
        dist_to_goal, yaw,
        orbit_.yaw_accumulator,
        orbiting, stalled, bs_val,
        stall_.bestDist(), stall_.sinceImprove(),
        closest_obs, best_idx,
        estop ? 1.0 : 0.0,
        obstacle_det ? 1.0 : 0.0,
        1.0,                                        // esdf_mode = true
        static_cast<double>(planner_->esdfVoxelCount())
    };
    diag_pub_->publish(diag);
  }

  std::unique_ptr<uav_local_planner::MotionPrimitives> planner_;
  StallDetector stall_;
  OrbitDetector orbit_;
  Eigen::Vector3d prev_cmd_{0, 0, 0};
  double max_accel_{0.75};
  double rate_hz_{20.0};
  double esdf_voxel_size_{0.05};

  Eigen::Vector3d pos_{0, 0, 0}, waypoint_{0, 0, 0};
  double yaw_{0.0};
  bool have_odom_{false}, have_waypoint_{false}, have_esdf_{false};

  std::mutex planner_mutex_, odom_mutex_, wp_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPEsdfNode>());
  rclcpp::shutdown();
}
