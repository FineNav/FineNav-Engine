// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/concepts.hpp"
#include "finenav_route/finenav_route_params.hpp"
#include "finenav_route/node_spatial_tree.hpp"
#include "finenav_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "finenav_route/route_planner.hpp"
#include "finenav_util/algo_configurator.hpp"

namespace finenav_route
{

class RouteGraphView
{
public:
  RouteGraphView() = default;
};

class GraphRoutePlanner
{
public:
  GraphRoutePlanner() = default;

  static constexpr finenav::IOProfile InputProfile = {
    .pose = false,
    .vel = false,
    .accel = false,
    .time = false
  };
  static constexpr finenav::IOProfile OutputProfile = {
    .pose = true,
    .vel = false,
    .accel = false,
    .time = false
  };

  using ConfigType = finenav_route::Params;
  using IMapView = RouteGraphView;
  using Context = finenav::PlanningContext<GraphRoutePlanner>;
  using Result = std::optional<finenav::OutputDataT<GraphRoutePlanner>>;

  void configure(const ConfigType & params);
  void reset();
  Result plan(const Context & ctx, const IMapView & map_view);

  const Graph & graph() const {return graph_;}
  const GraphToIDMap & idMap() const {return id_to_graph_map_;}

protected:
  bool ensureGraphLoaded();
  unsigned int findNearestNode(const geometry_msgs::msg::Pose & pose);
  Result makeNextNodeTarget(const Route & route, unsigned int start_index);
  Route pruneStartAndGoal(const Route & input_route, const RouteRequest & request) const;

  ConfigType params_;
  bool configured_{false};
  bool graph_loaded_{false};
  Graph graph_;
  GraphToIDMap id_to_graph_map_;
  NodeSpatialTree node_spatial_tree_;
  RoutePlanner route_planner_;
};

}  // namespace finenav_route

FINENAV_REGISTER_ALGO_CONFIG(finenav_route::GraphRoutePlanner, finenav_route)
