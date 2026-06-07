// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route_path_search.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────
//
// Called whenever a dynamic parameter changes.
// Reloads the graph only when graph_file actually changes.

void RoutePathSearch::configure(const ConfigType& params)
{
    path_density_ = params.path_density;

    if (params.graph_file == graph_file_) return;   // no change

    graph_file_   = params.graph_file;
    cached_graph_ = std::nullopt;

    if (graph_file_.empty()) return;

    std::ifstream f(graph_file_);
    if (!f.is_open()) {
        fprintf(stderr,
                "[RoutePathSearch] Cannot open graph_file: %s\n",
                graph_file_.c_str());
        return;
    }

    std::ostringstream buf;
    buf << f.rdbuf();
    cached_graph_ = route::loadFromJson(buf.str());

    if (!cached_graph_) {
        fprintf(stderr,
                "[RoutePathSearch] Failed to parse route graph: %s\n",
                graph_file_.c_str());
    }
}

// ── plan ───────────────────────────────────────────────────────────────────

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    if (!cached_graph_ || cached_graph_->nodes.empty()) return std::nullopt;
    const auto& graph = *cached_graph_;

    // Obstacle-aware BFS to snap start and goal to nearest reachable graph nodes
    const size_t start_idx = findNearestNodeIndex(graph, ctx.robot_state.pose, map_view);
    if (start_idx == SIZE_MAX) return std::nullopt;

    const size_t goal_idx = findNearestNodeIndex(graph, ctx.goal_pose, map_view);
    if (goal_idx == SIZE_MAX) return std::nullopt;

    // Dijkstra on the topological graph
    auto route_opt = route::findRoute(graph, start_idx, goal_idx, path_density_);
    if (!route_opt || route_opt->empty()) return std::nullopt;

    // Pack output — orientations are set by a higher layer
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
// Runs AStarAlgorithm::findPathToAny (h=0, Dijkstra) on the real occupancy
// map.  The search expands from the cell containing `pose` outward until it
// finds a cell that contains at least one route graph node.
//
// Using the real map means the connector is obstacle-aware: a node that can
// only be reached by going through a wall will not be selected.
//
// Cell size = map_view.getStepSize() (map resolution) so the BFS is in the
// same cell space as AstarPathSearch / BfsPathSearch.

size_t RoutePathSearch::findNearestNodeIndex(
    const route::Graph&             graph,
    const geometry_msgs::msg::Pose& pose,
    const IMapView&                 map_view) const
{
    const double step = map_view.getStepSize();

    auto toCell = [step](double x, double y) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            0
        };
    };
    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step, (c.y + 0.5) * step, 0.0 };
    };

    // Build cell → node-index map
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_nodes;
    const astar::Vec3 origin = toCell(pose.position.x, pose.position.y);

    int max_depth = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const astar::Vec3 c = toCell(graph.nodes[i].x, graph.nodes[i].y);
        cell_nodes[c].push_back(i);
        max_depth = std::max(max_depth,
            std::max(std::abs(c.x - origin.x), std::abs(c.y - origin.y)));
    }

    // Obstacle-aware lambdas — use real map data
    auto is_walkable = [&](const astar::Vec3& c) {
        return c.z == 0 &&
               std::abs(c.x - origin.x) <= max_depth &&
               std::abs(c.y - origin.y) <= max_depth &&
               map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) {
        return map_view.getCost(cellCentre(c));
    };
    auto is_goal = [&](const astar::Vec3& c) {
        return cell_nodes.contains(c);
    };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(5.0); // TODO: 后面这些应该也成为动态参数

    const auto path = algo.findPathToAny(origin, is_goal);
    if (!path || path->empty()) return SIZE_MAX;

    // Among nodes in the found cell, pick the one geometrically closest to pose
    const auto& candidates = cell_nodes.at(path->back());
    size_t best_idx  = SIZE_MAX;
    double best_dist = std::numeric_limits<double>::infinity();

    for (const size_t idx : candidates) {
        const double d = std::hypot(graph.nodes[idx].x - pose.position.x,
                                    graph.nodes[idx].y - pose.position.y);
        if (d < best_dist ||
            (std::abs(d - best_dist) < 1e-9 &&
             graph.nodes[idx].id < graph.nodes[best_idx].id))
        {
            best_idx  = idx;
            best_dist = d;
        }
    }
    return best_idx;
}

} // namespace finenav

