// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route_path_search.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <unordered_map>

namespace finenav {

void RoutePathSearch::configure(const ConfigType& params)
{
    connector_density_ = params.connector_density;
    path_density_      = params.path_density;
}

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&    ctx,
    const IGraphView& graph_view)
{
    // 1. Parse graph
    auto graph_opt = route::loadFromJson(graph_view.getGraphJson());
    if (!graph_opt || graph_opt->nodes.empty()) return std::nullopt;
    const auto& graph = *graph_opt;

    // 2. Nearest route node to start pose
    const size_t start_idx = findNearestNodeIndex(graph, ctx.robot_state.pose);
    if (start_idx == SIZE_MAX) return std::nullopt;

    // 3. Nearest route node to goal pose
    const size_t goal_idx = findNearestNodeIndex(graph, ctx.goal_pose);
    if (goal_idx == SIZE_MAX) return std::nullopt;

    // 4. Dijkstra on the graph
    auto route_opt = route::findRoute(graph, start_idx, goal_idx, path_density_);
    if (!route_opt || route_opt->empty()) return std::nullopt;

    // 5. Pack into FineNav output (identity orientation — higher layers set headings)
    OutputDataT<RoutePathSearch> output;
    output.poses.reserve(route_opt->size());
    for (auto [x, y] : *route_opt) {
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }
    return output;
}

// ── findNearestNodeIndex ───────────────────────────────────────────────────
//
// Runs AStarAlgorithm::findPathToAny (h=0 = Dijkstra) on a virtual cell grid
// whose cells are connector_density_ metres wide.  The "goal" predicate is
// satisfied by any cell that contains at least one route node.
//
// Among all nodes in the found cell, we return the one geometrically closest
// to `pose`.

size_t RoutePathSearch::findNearestNodeIndex(
    const route::Graph&             graph,
    const geometry_msgs::msg::Pose& pose) const
{
    const double res = connector_density_;

    auto toCell = [res](double x, double y) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / res)),
            static_cast<int>(std::floor(y / res)),
            0
        };
    };

    // Map each cell to the node indices it contains.
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_nodes;
    const astar::Vec3 origin = toCell(pose.position.x, pose.position.y);

    int max_depth = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const astar::Vec3 c = toCell(graph.nodes[i].x, graph.nodes[i].y);
        cell_nodes[c].push_back(i);
        max_depth = std::max(max_depth,
            std::max(std::abs(c.x - origin.x), std::abs(c.y - origin.y)));
    }

    // BFS outward until we hit a cell that contains a route node.
    auto is_walkable = [&](const astar::Vec3& c) {
        return c.z == 0 &&
               std::abs(c.x - origin.x) <= max_depth &&
               std::abs(c.y - origin.y) <= max_depth;
    };
    auto get_cost = [](const astar::Vec3&) { return 0.0; };
    auto is_goal  = [&](const astar::Vec3& c) { return cell_nodes.contains(c); };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(5.0);

    const auto path = algo.findPathToAny(origin, is_goal);
    if (!path || path->empty()) return SIZE_MAX;

    // Pick the node in the found cell that is closest to pose.
    const auto& candidates = cell_nodes.at(path->back());
    size_t best_idx  = SIZE_MAX;
    double best_dist = std::numeric_limits<double>::infinity();

    for (const size_t idx : candidates) {
        const double d = std::hypot(graph.nodes[idx].x - pose.position.x,
                                    graph.nodes[idx].y - pose.position.y);
        if (d < best_dist ||
            (std::abs(d - best_dist) < 1e-9 && graph.nodes[idx].id < graph.nodes[best_idx].id))
        {
            best_idx  = idx;
            best_dist = d;
        }
    }
    return best_idx;
}

} // namespace finenav

