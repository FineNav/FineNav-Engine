// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_mppi_controller/critics/path_rotation_critic.hpp"

#include <cmath>
#include <limits>

#include "tf2/utils.h"

#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

void PathRotationCritic::initialize(const finenav_mppi_controller::Params & params)
{
  logger_ = rclcpp::get_logger("PathRotationCritic");
  const auto & critic_params = params.PathRotationCritic;
  enabled_ = critic_params.enabled;
  power_ = static_cast<unsigned int>(critic_params.cost_power);
  weight_ = static_cast<float>(critic_params.cost_weight);
}

void PathRotationCritic::score(
  CriticData & data,
  const nav2_mppi_controller::IMapView &)
{
  if (!enabled_ || data.path.x.size() == 0 || data.path.yaws.size() == 0 ||
    data.state.wz.cols() == 0)
  {
    return;
  }

  const float robot_x = static_cast<float>(data.state.pose.pose.position.x);
  const float robot_y = static_cast<float>(data.state.pose.pose.position.y);
  float closest_distance_sq = std::numeric_limits<float>::max();
  Eigen::Index closest_path_index = 0;
  for (Eigen::Index i = 0; i < data.path.x.size(); ++i) {
    const float dx = data.path.x(i) - robot_x;
    const float dy = data.path.y(i) - robot_y;
    const float distance_sq = dx * dx + dy * dy;
    if (distance_sq < closest_distance_sq) {
      closest_distance_sq = distance_sq;
      closest_path_index = i;
    }
  }

  const float robot_yaw = static_cast<float>(tf2::getYaw(data.state.pose.pose.orientation));
  const float heading_error = utils::shortest_angular_distance(
    robot_yaw, data.path.yaws(closest_path_index));
  if (heading_error == 0.0F) {
    return;
  }

  const bool should_rotate_positive = heading_error > 0.0F;
  const float time_steps = static_cast<float>(data.state.wz.cols());
  for (Eigen::Index trajectory = 0; trajectory < data.state.wz.rows(); ++trajectory) {
    const bool linear_velocity_is_zero =
      (data.state.vx.row(trajectory) == 0.0F).all() &&
      (data.state.vy.row(trajectory) == 0.0F).all();
    if (!linear_velocity_is_zero) {
      continue;
    }

    Eigen::Index opposite_direction_count = 0;
    for (Eigen::Index step = 0; step < data.state.wz.cols(); ++step) {
      const float angular_velocity = data.state.wz(trajectory, step);
      if ((should_rotate_positive && angular_velocity < 0.0F) ||
        (!should_rotate_positive && angular_velocity > 0.0F))
      {
        ++opposite_direction_count;
      }
    }

    if (opposite_direction_count > 0) {
      const float opposite_direction_ratio =
        static_cast<float>(opposite_direction_count) / time_steps;
      data.costs(trajectory) += std::pow(opposite_direction_ratio * weight_, power_);
    }
  }

  RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics
