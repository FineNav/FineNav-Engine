// Copyright (c) 2025 Open Navigation LLC
// Adapted for FineNav by IWIN-FINS Lab, Shanghai Jiao Tong University.
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

#ifndef FINENAV_ROUTE__PATH_CONVERTER_HPP_
#define FINENAV_ROUTE__PATH_CONVERTER_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "finenav_core/plan/data_packet.hpp"
#include "finenav_route/corner_smoothing.hpp"
#include "finenav_route/types.hpp"

namespace finenav_route
{

class PathConverter
{
public:
  struct Config
  {
    float density{0.05f};
    float smoothing_radius{1.0f};
    bool smooth_corners{false};
  };

  static constexpr finenav::IOProfile OutputProfile = {
    .pose = true,
    .vel = false,
    .accel = false,
    .time = false
  };

  using Trajectory = finenav::DataPacket<OutputProfile>;

  PathConverter() = default;
  ~PathConverter() = default;

  void configure(const Config & config);

  Trajectory densify(
    const Route & route,
    const ReroutingState & rerouting_info = ReroutingState()) const;

  void interpolateEdge(
    float x0, float y0, float x1, float y1,
    std::vector<geometry_msgs::msg::Pose> & poses) const;
  void appendEdgePolyline(
    const DirectionalEdge & edge,
    std::vector<geometry_msgs::msg::Pose> & poses) const;

protected:
  geometry_msgs::msg::Pose makePose(float x, float y) const;
  void assignOrientations(std::vector<geometry_msgs::msg::Pose> & poses) const;

  float density_{0.05f};
  float smoothing_radius_{1.0f};
  bool smooth_corners_{false};
};

}  // namespace finenav_route

#endif  // FINENAV_ROUTE__PATH_CONVERTER_HPP_
