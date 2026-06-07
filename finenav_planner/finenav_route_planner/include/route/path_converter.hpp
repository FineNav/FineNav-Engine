// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "route/types.hpp"

namespace finenav_route
{

class PathConverter
{
public:
  struct Config { float density{0.05f}; };

  struct Trajectory { std::vector<geometry_msgs::msg::Pose> poses; };

  PathConverter() = default;
  ~PathConverter() = default;

  void configure(const Config & config);

  Trajectory densify(
    const Route & route,
    const ReroutingState & rerouting_info = ReroutingState()) const;

  std::vector<geometry_msgs::msg::Pose> extractTopologicalPoses(const Route & route) const;

  void interpolateEdge(
    float x0, float y0, float x1, float y1,
    std::vector<geometry_msgs::msg::Pose> & poses) const;

  void appendEdgePolyline(
    const DirectionalEdge & edge,
    std::vector<geometry_msgs::msg::Pose> & poses) const;

protected:
  geometry_msgs::msg::Pose makePose(float x, float y) const;
  void appendPoseIfNew(
    const geometry_msgs::msg::Pose & pose,
    std::vector<geometry_msgs::msg::Pose> & poses) const;
  void assignOrientations(std::vector<geometry_msgs::msg::Pose> & poses) const;

  float density_{0.05f};
};

}  // namespace finenav_route

