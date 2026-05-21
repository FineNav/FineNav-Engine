// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route/graph_route_planner.hpp"

#include <cmath>

#include "finenav_route/route_exceptions.hpp"
#include "finenav_route/utils.hpp"

namespace finenav_route
{

namespace
{
constexpr float kEpsilon = 1e-6f;

geometry_msgs::msg::PoseStamped toStamped(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.pose = pose;
  return stamped;
}
}  // namespace

void GraphRoutePlanner::configure(const ConfigType & params)
{
  params_ = params;

  RoutePlanner::Config planner_config;
  planner_config.max_iterations = params_.max_iterations;
  planner_config.distance_weight = static_cast<float>(params_.distance_weight);
  route_planner_.configure(planner_config);

  configured_ = true;
  graph_loaded_ = false;
}

void GraphRoutePlanner::reset()
{
  // Stateless across episodes. The graph cache is intentionally retained.
}

GraphRoutePlanner::Result GraphRoutePlanner::plan(
  const Context & ctx,
  const IMapView & /* map_view */)
{
  if (!configured_) {
    return std::nullopt;
  }

  if (!ensureGraphLoaded() || graph_.empty()) {
    return std::nullopt;
  }

  try {
    const unsigned int start_index = findNearestNode(ctx.robot_state.pose);
    const unsigned int goal_index = findNearestNode(ctx.goal_pose);

    Route route;
    RouteRequest request;
    request.start_nodeid = graph_.at(start_index).nodeid;
    request.goal_nodeid = graph_.at(goal_index).nodeid;
    request.start_pose = toStamped(ctx.robot_state.pose);
    request.goal_pose = toStamped(ctx.goal_pose);
    request.use_poses = true;

    if (start_index == goal_index) {
      route.start_node = &graph_.at(start_index);
      route.route_cost = 0.0f;
    } else {
      route = route_planner_.findRoute(graph_, start_index, goal_index, {}, request);
    }

    route = pruneStartAndGoal(route, request);
    return makeNextNodeTarget(route, start_index);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("GraphRoutePlanner"),
      "Route planning failed: %s", ex.what());
    return std::nullopt;
  }
}

bool GraphRoutePlanner::ensureGraphLoaded()
{
  if (graph_loaded_) {
    return true;
  }

  if (params_.graph_filepath.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("GraphRoutePlanner"), "No route graph file configured.");
    return false;
  }

  Graph graph;
  GraphToIDMap id_to_graph_map;
  GeoJsonGraphFileLoader loader;
  if (!loader.loadGraphFromFile(graph, id_to_graph_map, params_.graph_filepath)) {
    return false;
  }

  for (auto & node : graph) {
    if (node.coords.frame_id.empty()) {
      node.coords.frame_id = params_.route_frame;
    }
  }

  graph_ = std::move(graph);
  id_to_graph_map_ = std::move(id_to_graph_map);
  node_spatial_tree_.computeTree(graph_);
  graph_loaded_ = true;
  return true;
}

unsigned int GraphRoutePlanner::findNearestNode(const geometry_msgs::msg::Pose & pose)
{
  std::vector<unsigned int> candidates;
  geometry_msgs::msg::PoseStamped stamped = toStamped(pose);
  if (!node_spatial_tree_.findNearestGraphNodesToPose(stamped, candidates) || candidates.empty()) {
    throw IndeterminantNodesOnGraph("Could not determine nearest route graph node.");
  }
  return candidates.front();
}

GraphRoutePlanner::Result GraphRoutePlanner::makeNextNodeTarget(
  const Route & route,
  unsigned int start_index)
{
  finenav::OutputDataT<GraphRoutePlanner> output;
  const NodePtr next_node = route.edges.empty() ? &graph_[start_index] : route.edges.front()->end;

  if (!next_node) {
    return std::nullopt;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = next_node->coords.x;
  pose.position.y = next_node->coords.y;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  output.poses.push_back(pose);
  return output;
}

Route GraphRoutePlanner::pruneStartAndGoal(
  const Route & input_route,
  const RouteRequest & request) const
{
  Route pruned_route = input_route;
  if (input_route.edges.empty() || !request.use_poses) {
    return pruned_route;
  }

  NodePtr first = pruned_route.start_node;
  NodePtr next = pruned_route.edges[0]->end;
  float vrx = next->coords.x - first->coords.x;
  float vry = next->coords.y - first->coords.y;
  float vpx = request.start_pose.pose.position.x - first->coords.x;
  float vpy = request.start_pose.pose.position.y - first->coords.y;
  float dot_prod = utils::normalizedDot(vrx, vry, vpx, vpy);
  Coordinates closest_pt_on_edge = utils::findClosestPoint(
    request.start_pose, first->coords, next->coords);

  if (dot_prod > kEpsilon &&
    hypotf(vpx, vpy) > static_cast<float>(params_.min_prune_dist_from_start) &&
    utils::distance(closest_pt_on_edge, request.start_pose) <=
    static_cast<float>(params_.max_prune_dist_from_edge))
  {
    pruned_route.start_node = next;
    pruned_route.route_cost -= pruned_route.edges.front()->end->search_state.traversal_cost;
    pruned_route.edges.erase(pruned_route.edges.begin());
  }

  if (!params_.prune_goal || pruned_route.edges.empty()) {
    return pruned_route;
  }

  next = pruned_route.edges.back()->start;
  NodePtr last = pruned_route.edges.back()->end;
  vrx = last->coords.x - next->coords.x;
  vry = last->coords.y - next->coords.y;
  vpx = request.goal_pose.pose.position.x - last->coords.x;
  vpy = request.goal_pose.pose.position.y - last->coords.y;

  dot_prod = utils::normalizedDot(vrx, vry, vpx, vpy);
  closest_pt_on_edge = utils::findClosestPoint(request.goal_pose, next->coords, last->coords);
  if (dot_prod < -kEpsilon &&
    hypotf(vpx, vpy) > static_cast<float>(params_.min_prune_dist_from_goal) &&
    utils::distance(closest_pt_on_edge, request.goal_pose) <=
    static_cast<float>(params_.max_prune_dist_from_edge))
  {
    pruned_route.route_cost -= pruned_route.edges.back()->end->search_state.traversal_cost;
    pruned_route.edges.pop_back();
  }

  return pruned_route;
}

}  // namespace finenav_route
