// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_rotate_shim/rotate_shim.hpp"

#include <algorithm>
#include <cmath>

namespace finenav {

// ==============================================================================
// Configure
// ==============================================================================
void RotateShim::configure(const ConfigType& params)
{
  forward_sampling_distance_ = params.forward_sampling_distance;
  angular_speed_             = params.angular_speed;
  angular_dist_threshold_    = params.angular_dist_threshold;
  close_to_goal_threshold_   = params.close_to_goal_threshold;
}

// ==============================================================================
// Reset
// ==============================================================================
void RotateShim::reset()
{
  // Nothing persistent to reset between episodes
}

// ==============================================================================
// Plan
// ==============================================================================
std::optional<DataPacket<RotateShim::OutputProfile>> RotateShim::plan(
    const Context& ctx,
    const IMapView& /*map_view*/)
{
  const auto& ref_traj  = ctx.ref_traj;
  const auto& robot_pos = ctx.robot_state.pose.position;
  const auto& goal_pos  = ctx.goal_pose.position;

  // 计算到 goal 的距离
  const double dist_to_goal = std::hypot(goal_pos.x - robot_pos.x,
                                         goal_pos.y - robot_pos.y);

  const double current_yaw = yawFromQuat(ctx.robot_state.pose.orientation);
  double target_yaw;

  // ── 分支 1: 足够靠近 goal → 直接对齐 goal_pose 的朝向 ──
  if (dist_to_goal <= close_to_goal_threshold_) {
    target_yaw = yawFromQuat(ctx.goal_pose.orientation);
  }
  // ── 分支 2: 离 goal 还远 → 对齐路径前瞻点（原有逻辑） ──
  else {
    if (ref_traj.poses.empty()) {
      return std::nullopt;
    }
    const auto sampled_pt = getSampledPathPt(ref_traj.poses, forward_sampling_distance_);
    const double dx = sampled_pt.x - robot_pos.x;
    const double dy = sampled_pt.y - robot_pos.y;
    target_yaw = std::atan2(dy, dx);
  }

  // 计算 angular distance 并判断是否已对齐
  const double angular_distance = normalizeAngle(target_yaw - current_yaw);
  if (std::abs(angular_distance) < angular_dist_threshold_) {
    return std::nullopt;
  }

  // 未对齐 — 生成纯旋转轨迹
  DataPacket<RotateShim::OutputProfile> output;
  const double sign = (angular_distance > 0.0) ? 1.0 : -1.0;

  output.poses.push_back(ctx.robot_state.pose);

  geometry_msgs::msg::Twist twist;
  twist.linear.x   = 0.0;
  twist.linear.y   = 0.0;
  twist.linear.z   = 0.0;
  twist.angular.x  = 0.0;
  twist.angular.y  = 0.0;
  twist.angular.z  = sign * angular_speed_;
  output.twists.push_back(twist);

  return output;
}

// ==============================================================================
// getSampledPathPt — 参照 Nav2 RotationShimController::getSampledPathPt
// ==============================================================================
geometry_msgs::msg::Point RotateShim::getSampledPathPt(
    const std::vector<geometry_msgs::msg::Pose>& ref_poses,
    double sampling_distance)
{
  if (ref_poses.size() < 2) {
    // 路径过短，直接返回唯一点的位置
    return ref_poses.front().position;
  }

  const auto& start = ref_poses.front().position;

  // 从第二点开始遍历，累加距离
  for (size_t i = 1; i < ref_poses.size(); ++i) {
    const double dx = ref_poses[i].position.x - start.x;
    const double dy = ref_poses[i].position.y - start.y;
    if (std::hypot(dx, dy) >= sampling_distance) {
      return ref_poses[i].position;
    }
  }

  // 路径不够长，返回最后一个点
  return ref_poses.back().position;
}

// ==============================================================================
// yawFromQuat
// ==============================================================================
double RotateShim::yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ==============================================================================
// normalizeAngle
// ==============================================================================
double RotateShim::normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

}  // namespace finenav
