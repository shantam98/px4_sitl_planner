#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

namespace uav_local_planner {

struct MPConfig {
  int    num_primitives   = 72;    // horizontal directions (360° / 5° = 72)
  double primitive_length = 2.0;  // metres — matches A010 max range
  double collision_radius = 0.45; // robot_radius (0.30) + safety_margin (0.15)
  double max_speed        = 2.5;  // m/s horizontal
  double min_speed        = 0.2;  // m/s — floor when close to obstacle
  double max_vz           = 1.0;  // m/s vertical
  double min_clearance    = 0.6;  // e-stop distance (metres)
  double w_goal           = 3.0;  // weight: angular distance to goal direction
  double w_prev           = 2.0;  // weight: angular distance to previous primitive
  double alt_kp           = 0.8;  // P-gain for altitude error → vz
};

struct MPResult {
  Eigen::Vector3d velocity;           // in map (ENU) frame
  bool   estop;
  bool   obstacle_detected;
  double closest_obstacle_dist;
  int    best_primitive;             // index into directions_, -1 if none valid
};

class MotionPrimitives {
public:
  explicit MotionPrimitives(const MPConfig& cfg);

  // Primary update — call at 20 Hz.
  // cloud     : merged ToF cloud in base_link frame
  // drone_pos : position in map frame (ENU)
  // drone_yaw : yaw angle in map frame (radians)
  // waypoint  : target in map frame (ENU)
  MPResult update(
      const pcl::PointCloud<pcl::PointXYZ>& cloud,
      const Eigen::Vector3d& drone_pos,
      double drone_yaw,
      const Eigen::Vector3d& waypoint);

  void reset();

private:
  // Minimum distance from point p (in base_link frame) to primitive segment
  // Segment runs from origin to direction_[i] * primitive_length.
  double pointToSegmentDist(
      const Eigen::Vector3d& p,
      const Eigen::Vector3d& seg_end) const;

  double adaptiveSpeed(double closest_dist) const;

  MPConfig cfg_;

  // Pre-computed horizontal unit-direction vectors (in base_link frame).
  // Index 0 = forward (+x), increasing counter-clockwise.
  std::vector<Eigen::Vector3d> directions_;

  int prev_primitive_{-1};
};

}  // namespace uav_local_planner
