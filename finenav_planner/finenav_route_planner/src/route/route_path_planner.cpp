// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route/route_path_planner.hpp"
#include "route/route_exceptions.hpp"

namespace finenav_route
{

RoutePathPlanner::RoutePathPlanner() { configure(Config{}); }

void RoutePathPlanner::configure(const Config & config)
{
  RoutePlanner::Config route_config;
  route_config.max_iterations  = config.max_iterations;
  route_config.distance_weight = config.distance_weight;
  route_planner_.configure(route_config);
  path_converter_.configure(config.path_converter);
}

RoutePathPlanner::PlanResult RoutePathPlanner::plan(
  const std::string & graph_json,
  unsigned int start_node_id,
  unsigned int goal_node_id)
{
  Graph graph;
  GraphToIDMap id_to_graph_map;
  if (!loader_.loadGraphFromJson(graph, id_to_graph_map, graph_json))
    throw NoValidGraph("Failed to load route graph json");

  const auto start_it = id_to_graph_map.find(start_node_id);
  if (start_it == id_to_graph_map.end())
    throw IndeterminantNodesOnGraph("Start node id does not exist in route graph");

  const auto goal_it = id_to_graph_map.find(goal_node_id);
  if (goal_it == id_to_graph_map.end())
    throw IndeterminantNodesOnGraph("Goal node id does not exist in route graph");

  Route route;
  RouteRequest request;
  request.start_nodeid = start_node_id;
  request.goal_nodeid  = goal_node_id;
  request.use_poses    = false;

  if (start_node_id == goal_node_id) {
    route.start_node = &graph.at(start_it->second);
    route.route_cost = 0.0f;
  } else {
    route = route_planner_.findRoute(
      graph, start_it->second, goal_it->second, {}, request);
  }

  PlanResult result;
  result.trajectory        = toMsg(path_converter_.densify(route));
  result.topological_poses = path_converter_.extractTopologicalPoses(route);
  return result;
}

unsigned int RoutePathPlanner::findNodeIdByFrame(
  const std::string & graph_json, const std::string & frame_name)
{
  Graph graph;
  GraphToIDMap id_to_graph_map;
  if (!loader_.loadGraphFromJson(graph, id_to_graph_map, graph_json))
    throw NoValidGraph("Failed to load route graph json");

  for (const auto & node : graph) {
    if (node.coords.frame_id == frame_name) return node.nodeid;
  }
  throw IndeterminantNodesOnGraph("Node with frame '" + frame_name + "' not found in graph.");
}

finenav_msgs::msg::Trajectory RoutePathPlanner::toMsg(const PathConverter::Trajectory & path) const
{
  finenav_msgs::msg::Trajectory msg;
  msg.valid_fields = finenav_msgs::msg::Trajectory::TRAJ_POSE;
  msg.poses = path.poses;
  return msg;
}

}  // namespace finenav_route

