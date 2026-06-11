// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critic_manager.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"

namespace mppi
{
using namespace nav2_mppi_controller;

void CriticManager::on_configure(
  std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
  const finenav_mppi_controller::Params & params)
{
  critics_ = std::move(critics);

  visualize_ = params.visualize;

  RCLCPP_DEBUG(rclcpp::get_logger("Critic Manager"), "Critic Manager configured");

  if (visualize_) {
    // Create a standalone node for publishing CriticsStats.
    // publish() works without spinning — no executor needed for publisher-only nodes.
    stats_node_ = std::make_shared<rclcpp::Node>("mppi_critic_stats");
    critic_stats_pub_ =
      stats_node_->create_publisher<finenav_msgs::msg::CriticsStats>("~/critics_stats", 10);

    RCLCPP_INFO(rclcpp::get_logger("Critic Manager"),
      "CriticsStats visualization enabled. Publishing to ~/critics_stats");
  }

  for (size_t q = 0; q < critics_.size(); q++) {
    critics_[q]->initialize(params);
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data, const IMapView & map_view)
{
  RCLCPP_DEBUG(rclcpp::get_logger("Critic Manager"), "Scoring...");

  if (!visualize_ || !critic_stats_pub_) {
    // Fast path: no visualization overhead
    for (size_t q = 0; q < critics_.size(); q++) {
      if (data.fail_flag) {
        break;
      }
      critics_[q]->score(data, map_view);
    }
    return;
  }

  // Visualization path: track per-critic cost contributions
  auto stats_msg = std::make_unique<finenav_msgs::msg::CriticsStats>();
  stats_msg->stamp = stats_node_->now();
  stats_msg->critics.reserve(critics_.size());
  stats_msg->changed.reserve(critics_.size());
  stats_msg->costs_sum.reserve(critics_.size());

  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }

    // Snapshot total costs before this critic modifies them
    float costs_before = xt::sum(data.costs, xt::evaluation_strategy::immediate)();

    critics_[q]->score(data, map_view);

    // Compute the cost contribution from this critic
    float costs_after = xt::sum(data.costs, xt::evaluation_strategy::immediate)();
    float cost_sum = costs_after - costs_before;

    stats_msg->critics.push_back(critics_[q]->getName());
    stats_msg->changed.push_back(std::abs(cost_sum) > 1e-6f);
    stats_msg->costs_sum.push_back(cost_sum);
  }

  critic_stats_pub_->publish(std::move(stats_msg));
}

}  // namespace mppi
