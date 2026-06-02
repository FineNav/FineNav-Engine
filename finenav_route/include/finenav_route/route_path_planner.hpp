// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_route/path_converter.hpp"
#include "finenav_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "finenav_route/route_planner.hpp"

namespace finenav_route
{

class RoutePathPlanner
{
public:
  struct Config
  {
    int max_iterations{0};
    float distance_weight{1.0f};
    PathConverter::Config path_converter;
  };

  RoutePathPlanner();

  void configure(const Config & config);

  finenav_msgs::msg::Trajectory plan(
    const std::string & graph_json,
    unsigned int start_node_id,
    unsigned int goal_node_id);

private:
  finenav_msgs::msg::Trajectory toMsg(const PathConverter::Trajectory & path) const;

  GeoJsonGraphFileLoader loader_;
  RoutePlanner route_planner_;
  PathConverter path_converter_;
};

}  // namespace finenav_route
