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

#include <chrono>

#include "finenav_mppi_controller/critic_manager.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"
#include "finenav_mppi_controller/timing_statistics.hpp"

namespace mppi
{
using namespace nav2_mppi_controller;

void CriticManager::on_configure(
    std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
    const finenav_mppi_controller::Params & params)
{
    critics_ = std::move(critics);
    RCLCPP_DEBUG(rclcpp::get_logger("Critic Manager"), "Critic Manager configured;");
    for (size_t q = 0; q < critics_.size(); q++) {
        critics_[q]->initialize(params);
    }
}

//void CriticManager::getParams()
//{
//  auto node = parent_.lock();
//  auto getParam = parameters_handler_->getParamGetter(name_);
//  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
//}


std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data, const IMapView& map_view) const
{
    RCLCPP_DEBUG(rclcpp::get_logger("Critic Manager"), "Scoring...");
    for (size_t q = 0; q < critics_.size(); q++) {
        if (data.fail_flag) {
            break;
        }
        const auto score_start = std::chrono::steady_clock::now();
        critics_[q]->score(data,map_view);
        const double score_time_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - score_start).count();
        timingStatistics().addSample(critics_[q]->getName(), score_time_ms);
        RCLCPP_INFO(
          logger_, "[MPPIController] %s score time: %.3f ms",
          critics_[q]->getName().c_str(), score_time_ms);
    }
}

}  // namespace mppi
