// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "route/route_planner.hpp"

namespace finenav_route
{

void RoutePlanner::configure(const Config & config)
{
  max_iterations_ = config.max_iterations <= 0 ?
    std::numeric_limits<int>::max() : config.max_iterations;
  distance_weight_ = config.distance_weight;
}

Route RoutePlanner::findRoute(
  Graph & graph, unsigned int start_index, unsigned int goal_index,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  if (graph.empty()) throw NoValidGraph("Graph is invalid for routing!");

  const NodePtr & start_node = &graph.at(start_index);
  const NodePtr & goal_node  = &graph.at(goal_index);
  findShortestGraphTraversal(graph, start_node, goal_node, blocked_ids, route_request);

  EdgePtr parent_edge = goal_node->search_state.parent_edge;
  if (!parent_edge) throw NoValidRouteCouldBeFound("Could not find a route to the requested goal!");

  Route route;
  while (parent_edge) {
    route.edges.push_back(parent_edge);
    parent_edge = parent_edge->start->search_state.parent_edge;
  }
  std::reverse(route.edges.begin(), route.edges.end());
  route.start_node  = start_node;
  route.route_cost  = goal_node->search_state.integrated_cost;
  return route;
}

void RoutePlanner::resetSearchStates(Graph & graph)
{
  for (auto & node : graph) node.search_state.reset();
}

void RoutePlanner::findShortestGraphTraversal(
  Graph & graph, const NodePtr start_node, const NodePtr goal_node,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & route_request)
{
  resetSearchStates(graph);
  clearQueue();
  start_id_ = start_node->nodeid;
  goal_id_  = goal_node->nodeid;
  start_node->search_state.integrated_cost = 0.0;
  addNode(0.0, start_node);

  NodePtr neighbor{nullptr};
  EdgePtr edge{nullptr};
  float potential_cost = 0.0f;
  float traversal_cost = 0.0f;
  int   iterations = 0;

  while (!queue_.empty() && iterations < max_iterations_) {
    iterations++;
    auto [curr_cost, node] = getNextNode();
    if (curr_cost != node->search_state.integrated_cost) continue;
    if (isGoal(node)) { clearQueue(); return; }

    EdgeVector & edges = getEdges(node);
    for (auto & candidate_edge : edges) {
      edge     = &candidate_edge;
      neighbor = edge->end;
      if (!getTraversalCost(edge, traversal_cost, blocked_ids, route_request)) continue;
      potential_cost = curr_cost + traversal_cost;
      if (potential_cost < neighbor->search_state.integrated_cost) {
        neighbor->search_state.parent_edge       = edge;
        neighbor->search_state.integrated_cost   = potential_cost;
        neighbor->search_state.traversal_cost    = traversal_cost;
        addNode(potential_cost, neighbor);
      }
    }
  }
  if (iterations == max_iterations_) {
    clearQueue();
    throw TimedOut("Maximum iterations was exceeded!");
  }
}

bool RoutePlanner::getTraversalCost(
  const EdgePtr edge, float & score,
  const std::vector<unsigned int> & blocked_ids,
  const RouteRequest & /* route_request */)
{
  const auto is_blocked = std::find_if(blocked_ids.begin(), blocked_ids.end(),
    [&](unsigned int id){ return id == edge->edgeid || id == edge->end->nodeid; });
  if (is_blocked != blocked_ids.end()) return false;

  if (!edge->edge_cost.overridable) {
    if (edge->edge_cost.cost <= 0.0f)
      throw NoValidGraph("Edge " + std::to_string(edge->edgeid) +
                         " does not contain a valid fixed edge cost!");
    score = edge->edge_cost.cost;
    return true;
  }
  score = distance_weight_ * edge->getEdgeLength();
  return true;
}

NodeElement  RoutePlanner::getNextNode()        { auto d = queue_.top(); queue_.pop(); return d; }
void         RoutePlanner::addNode(const float cost, const NodePtr node) { queue_.emplace(cost, node); }
EdgeVector & RoutePlanner::getEdges(const NodePtr node) { return node->neighbors; }
void         RoutePlanner::clearQueue()          { NodeQueue q; std::swap(queue_, q); }
bool         RoutePlanner::isGoal(const NodePtr node)  { return node->nodeid == goal_id_; }
bool         RoutePlanner::isStart(const NodePtr node) { return node->nodeid == start_id_; }

EdgeType RoutePlanner::classifyEdge(const EdgePtr edge) {
  if (isStart(edge->start)) return EdgeType::START;
  if (isGoal(edge->end))    return EdgeType::END;
  return EdgeType::NONE;
}

}  // namespace finenav_route

