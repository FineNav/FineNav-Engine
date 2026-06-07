// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route_path_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>

#include "route/route_exceptions.hpp"

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────

void RoutePathSearch::configure(const ConfigType& params)
{
    connector_density_ = params.connector_density;

    finenav_route::RoutePlanner::Config route_cfg;
    route_cfg.max_iterations  = params.max_iterations;
    route_cfg.distance_weight = static_cast<float>(params.distance_weight);
    route_planner_.configure(route_cfg);

    finenav_route::PathConverter::Config pc_cfg;
    pc_cfg.density = static_cast<float>(params.path_density);
    path_converter_.configure(pc_cfg);
}

// ── plan ───────────────────────────────────────────────────────────────────

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&     ctx,
    const IGraphView&  graph_view)
{
    // 1. Load graph — single parse for the entire plan() call
    finenav_route::Graph        graph;
    finenav_route::GraphToIDMap id_map;
    if (!loader_.loadGraphFromJson(graph, id_map, graph_view.getGraphJson()))
        return std::nullopt;

    if (graph.empty()) return std::nullopt;

    // 2. BFS: nearest route node to start pose
    const NodeCandidate start_candidate = findNearestNode(graph, ctx.robot_state.pose);

    // 3. BFS: nearest route node to goal pose
    const NodeCandidate goal_candidate = findNearestNode(graph, ctx.goal_pose);

    // 4. Dijkstra on the graph (uses already-loaded graph — no second JSON parse)
    finenav_route::Route route;
    try {
        finenav_route::RouteRequest request;
        request.start_nodeid = start_candidate.node_id;
        request.goal_nodeid  = goal_candidate.node_id;
        request.use_poses    = false;

        if (start_candidate.node_id == goal_candidate.node_id) {
            route.start_node = &graph.at(start_candidate.graph_index);
            route.route_cost = 0.0f;
        } else {
            route = route_planner_.findRoute(
                graph,
                start_candidate.graph_index,
                goal_candidate.graph_index,
                {}, request);
        }
    } catch (const finenav_route::RouteException&) {
        return std::nullopt;
    }

    // 5. Densify and return
    const auto traj = path_converter_.densify(route);
    if (traj.poses.empty()) return std::nullopt;

    OutputDataT<RoutePathSearch> output;
    output.poses = traj.poses;
    return output;
}

// ── findNearestNode ────────────────────────────────────────────────────────
//
// BFS on a cell grid to find the route graph node closest to `pose`.
// Cell size = connector_density_ (configured from route.yaml).

RoutePathSearch::NodeCandidate RoutePathSearch::findNearestNode(
    const finenav_route::Graph&       graph,
    const geometry_msgs::msg::Pose&   pose) const
{
    if (graph.empty())
        throw std::runtime_error("[RoutePathSearch] Empty graph in findNearestNode");

    const double res = connector_density_;
    if (res <= 0.0 || !std::isfinite(res))
        throw std::runtime_error("[RoutePathSearch] connector_density must be positive");

    // Build cell → node index list
    auto toCell = [res](double x, double y) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / res)),
            static_cast<int>(std::floor(y / res)),
            0
        };
    };

    std::unordered_map<astar::Vec3, std::vector<unsigned int>, astar::Vec3Hash> nodes_by_cell;
    const astar::Vec3 start_cell = toCell(pose.position.x, pose.position.y);

    int max_depth = 0;
    for (unsigned int idx = 0; idx < graph.size(); ++idx) {
        const auto cell = toCell(graph[idx].coords.x, graph[idx].coords.y);
        nodes_by_cell[cell].push_back(idx);
        max_depth = std::max(max_depth,
            std::max(std::abs(cell.x - start_cell.x),
                     std::abs(cell.y - start_cell.y)));
    }

    // BFS: walk outward until we hit a cell that contains a route node
    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return c.z == 0 &&
               std::abs(c.x - start_cell.x) <= max_depth &&
               std::abs(c.y - start_cell.y) <= max_depth;
    };
    auto is_goal = [&](const astar::Vec3& c) -> bool {
        return nodes_by_cell.contains(c);
    };

    // BFS on a virtual cell grid = A* with h=0 (Dijkstra), uniform cost (get_cost=0).
    // We don't have an obstacle map here — is_walkable handles bounds only.
    auto get_cost_zero = [](const astar::Vec3&) { return 0.0; };
    astar::AStarAlgorithm algo(is_walkable, get_cost_zero);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(5.0);

    const auto path = algo.findPathToAny(start_cell, is_goal);
    if (!path || path->empty())
        throw std::runtime_error("[RoutePathSearch] BFS nearest-node search failed");

    // Among all nodes in the found cell, pick the one closest to pose
    const auto cell_it = nodes_by_cell.find(path->back());
    if (cell_it == nodes_by_cell.end())
        throw std::runtime_error("[RoutePathSearch] BFS result cell has no nodes (bug)");

    NodeCandidate best;
    best.distance = std::numeric_limits<double>::infinity();

    for (const unsigned int idx : cell_it->second) {
        const double d = std::hypot(
            static_cast<double>(graph[idx].coords.x) - pose.position.x,
            static_cast<double>(graph[idx].coords.y) - pose.position.y);
        if (d < best.distance ||
            (std::abs(d - best.distance) <= 1e-9 && graph[idx].nodeid < best.node_id))
        {
            best.node_id     = graph[idx].nodeid;
            best.graph_index = idx;
            best.distance    = d;
        }
    }

    return best;
}

} // namespace finenav


