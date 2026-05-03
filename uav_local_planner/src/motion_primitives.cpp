// motion_primitives.cpp
// Motion Primitive Library local planner.
//
// Replaces OctoMap-based VFH3D with direct ToF point cloud collision checking.
// Primitives are horizontal straight-line rays in the base_link frame —
// valid for purely-horizontal sensor arrays where vertical coverage is absent.

#include <uav_local_planner/motion_primitives.hpp>
#include <algorithm>
#include <limits>

namespace uav_local_planner {

MotionPrimitives::MotionPrimitives(const MPConfig& cfg) : cfg_(cfg)
{
  // Pre-compute evenly-spaced horizontal unit vectors in base_link frame.
  // Index 0 = drone forward (+x), increasing counter-clockwise.
  directions_.reserve(cfg_.num_primitives);
  for (int i = 0; i < cfg_.num_primitives; ++i) {
    double angle = i * 2.0 * M_PI / cfg_.num_primitives;
    directions_.emplace_back(std::cos(angle), std::sin(angle), 0.0);
  }
}

void MotionPrimitives::reset()
{
  prev_primitive_ = -1;
}

// ── Main update ───────────────────────────────────────────────────────────
MPResult MotionPrimitives::update(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const Eigen::Vector3d& drone_pos,
    double drone_yaw,
    const Eigen::Vector3d& waypoint)
{
  MPResult result;
  result.estop                 = false;
  result.obstacle_detected     = false;
  result.closest_obstacle_dist = std::numeric_limits<double>::max();
  result.best_primitive        = -1;
  result.velocity              = Eigen::Vector3d::Zero();

  // ── 1. Filter cloud and find closest obstacle ─────────────────────────
  // Cloud is in base_link (drone-centered). Drone center = origin.
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(cloud.size());

  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    Eigen::Vector3d pt(p.x, p.y, p.z);
    double d = pt.norm();
    if (d < 0.1 || d > cfg_.primitive_length + cfg_.collision_radius) continue;
    if (d < result.closest_obstacle_dist) result.closest_obstacle_dist = d;
    pts.push_back(pt);
  }

  result.obstacle_detected = result.closest_obstacle_dist < cfg_.primitive_length;

  // ── 2. E-stop ──────────────────────────────────────────────────────────
  if (result.closest_obstacle_dist < cfg_.min_clearance) {
    result.estop = true;
    return result;
  }

  // ── 3. Collision check — invalidate primitives ─────────────────────────
  std::vector<bool> valid(cfg_.num_primitives, true);
  for (const auto& pt : pts) {
    for (int i = 0; i < cfg_.num_primitives; ++i) {
      if (!valid[i]) continue;
      Eigen::Vector3d seg_end = directions_[i] * cfg_.primitive_length;
      if (pointToSegmentDist(pt, seg_end) < cfg_.collision_radius) {
        valid[i] = false;
      }
    }
  }

  // ── 4. Score valid primitives ──────────────────────────────────────────
  // Goal direction in base_link frame: rotate map-frame goal vec by -yaw.
  Eigen::Vector3d goal_map(waypoint.x() - drone_pos.x(),
                           waypoint.y() - drone_pos.y(), 0.0);
  if (goal_map.norm() < 1e-3) goal_map = Eigen::Vector3d(1.0, 0.0, 0.0);
  goal_map.normalize();

  double cy = std::cos(drone_yaw), sy = std::sin(drone_yaw);
  Eigen::Vector3d goal_base(
       cy * goal_map.x() + sy * goal_map.y(),
      -sy * goal_map.x() + cy * goal_map.y(),
       0.0);

  double best_cost = std::numeric_limits<double>::max();
  for (int i = 0; i < cfg_.num_primitives; ++i) {
    if (!valid[i]) continue;

    // Angular cost to goal (dot product → angle)
    double cos_goal = std::clamp(directions_[i].dot(goal_base), -1.0, 1.0);
    double d_goal   = std::acos(cos_goal);

    // History smoothness cost
    double d_prev = 0.0;
    if (prev_primitive_ >= 0) {
      double cos_prev = std::clamp(
          directions_[i].dot(directions_[prev_primitive_]), -1.0, 1.0);
      d_prev = std::acos(cos_prev);
    }

    double cost = cfg_.w_goal * d_goal + cfg_.w_prev * d_prev;
    if (cost < best_cost) {
      best_cost             = cost;
      result.best_primitive = i;
    }
  }

  // ── 5. Compute output velocity in map frame ───────────────────────────
  if (result.best_primitive < 0) {
    // All primitives blocked — hover
    return result;
  }

  prev_primitive_ = result.best_primitive;

  // Rotate selected direction from base_link → map frame
  const Eigen::Vector3d& dir_base = directions_[result.best_primitive];
  double speed = adaptiveSpeed(result.closest_obstacle_dist);

  double vx_map = (cy * dir_base.x() - sy * dir_base.y()) * speed;
  double vy_map = (sy * dir_base.x() + cy * dir_base.y()) * speed;

  // Altitude: proportional controller on height error (z same in base_link and map for ENU)
  double alt_err = waypoint.z() - drone_pos.z();
  double vz      = std::clamp(cfg_.alt_kp * alt_err, -cfg_.max_vz, cfg_.max_vz);

  result.velocity = Eigen::Vector3d(vx_map, vy_map, vz);
  return result;
}

// ── Helpers ───────────────────────────────────────────────────────────────
double MotionPrimitives::pointToSegmentDist(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& seg_end) const
{
  // Segment: 0 → seg_end.  t ∈ [0,1] parameterises the closest point.
  double len2 = seg_end.squaredNorm();
  if (len2 < 1e-10) return p.norm();
  double t       = std::clamp(p.dot(seg_end) / len2, 0.0, 1.0);
  return (p - t * seg_end).norm();
}

double MotionPrimitives::adaptiveSpeed(double closest_dist) const
{
  // Linear ramp: full speed beyond primitive_length/2, min speed at min_clearance.
  double d_high = cfg_.primitive_length / 2.0;
  double d_low  = cfg_.min_clearance;
  double t = std::clamp((closest_dist - d_low) / (d_high - d_low), 0.0, 1.0);
  return cfg_.min_speed + t * (cfg_.max_speed - cfg_.min_speed);
}

}  // namespace uav_local_planner
