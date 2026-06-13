// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route_path_search.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <mutex>

#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────
//
// Called whenever a dynamic parameter changes.
// Reloads the graph only when graph_file actually changes.

void RoutePathSearch::configure(const ConfigType& params)
{
    path_density_ = params.path_density;
}

// ── updateGraph ────────────────────────────────────────────────────────────

void RoutePathSearch::updateGraph(const std::string& geojson_str)
{
    std::lock_guard<std::mutex> lock(*graph_mutex_);
    cached_graph_ = route::loadFromJson(geojson_str);

    if (!cached_graph_) {
        RCLCPP_ERROR(
            rclcpp::get_logger("RoutePathSearch"),
            "Failed to parse route graph from /geojson topic.");
    } else {
        RCLCPP_INFO(
            rclcpp::get_logger("RoutePathSearch"),
            "Successfully loaded route graph from /geojson topic with %zu nodes.",
            cached_graph_->nodes.size());
    }
}

// ── plan ───────────────────────────────────────────────────────────────────

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    std::lock_guard<std::mutex> lock(*graph_mutex_);

    if (!cached_graph_ || cached_graph_->nodes.empty()) return std::nullopt;
    const auto& graph = *cached_graph_;

    // Obstacle-aware BFS to snap start and goal to nearest reachable graph nodes.
    // Each SnapResult also carries the world-frame connector path (pose → node).
    const auto start_snap = findNearestNodeIndex(graph, ctx.robot_state.pose, map_view);
    if (start_snap.node_index == SIZE_MAX) return std::nullopt;

    const auto goal_snap = findNearestNodeIndex(graph, ctx.goal_pose, map_view);
    if (goal_snap.node_index == SIZE_MAX) return std::nullopt;

    // Dijkstra on the topological graph
    auto route_opt = route::findRoute(graph, start_snap.node_index, goal_snap.node_index, path_density_);
    if (!route_opt || route_opt->empty()) return std::nullopt;

    OutputDataT<RoutePathSearch> output;
    output.poses.reserve(start_snap.connector_path.size()
                       + goal_snap.connector_path.size()
                       + route_opt->size());

    auto appendPose = [&](double x, double y) {
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    };

    auto isDegenerate = [](const std::vector<Position3D>& conn) -> bool {
        if (conn.size() < 2) return true;
        const double d = (conn.front() - conn.back()).norm();
        return d < 1e-6;
    };

    // ── Start connector (robot_pose → start_node) ──
    if (!isDegenerate(start_snap.connector_path)) {
        for (const auto& pt : start_snap.connector_path)
            appendPose(pt.x(), pt.y());
    }

    // ── Topological route ──
    // If we already added a start connector whose last point IS the start node,
    // skip the first route point when it coincides to avoid a duplicate.
    bool skip_first_route = false;
    if (!isDegenerate(start_snap.connector_path) && !output.poses.empty()) {
        const auto& last_conn = output.poses.back().position;
        const auto& first_route_x = (*route_opt)[0].first;
        const auto& first_route_y = (*route_opt)[0].second;
        skip_first_route = (std::hypot(last_conn.x - first_route_x,
                                        last_conn.y - first_route_y) < 1e-6);
    }

    for (size_t i = (skip_first_route ? 1 : 0); i < route_opt->size(); ++i)
        appendPose((*route_opt)[i].first, (*route_opt)[i].second);

    // ── Goal connector (goal_node → goal_pose), reversed ──
    if (!isDegenerate(goal_snap.connector_path)) {
        // Last point of route ≈ goal_node position; skip if coincident.
        const auto& route_end = output.poses.back().position;
        const auto& goal_last = goal_snap.connector_path.back();  // goal node
        const bool route_ends_at_node =
            std::hypot(route_end.x - goal_last.x(),
                       route_end.y - goal_last.y()) < 1e-6;

        // Walk backwards: goal_node → ... → goal_pose
        const size_t start_i = route_ends_at_node
            ? goal_snap.connector_path.size() - 2   // skip duplicate node
            : goal_snap.connector_path.size() - 1;

        for (size_t i = start_i; i != SIZE_MAX; --i)
            appendPose(goal_snap.connector_path[i].x(),
                       goal_snap.connector_path[i].y());
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
//
// Returns a SnapResult with both the node index and the world-frame connector
// path from `pose` to that node.

RoutePathSearch::SnapResult RoutePathSearch::findNearestNodeIndex(
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
    if (!path || path->empty()) return {SIZE_MAX, {}};

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

    // ── Build world-frame connector path (pose → nearest node) ──────────
    // The grid path runs from origin (robot/goal cell) to the cell
    // containing the route node.  Convert it to world coordinates,
    // anchoring the first point at the exact continuous pose and the
    // last point at the exact node position.
    std::vector<Position3D> connector;
    connector.reserve(path->size() + 1);

    // First point: exact continuous pose
    connector.push_back({pose.position.x, pose.position.y, 0.0});

    // Interior grid cells → cell centres (skip origin if it coincides)
    const bool skip_first = (path->size() >= 2 && (*path)[0] == origin);
    for (size_t i = (skip_first ? 1 : 0); i < path->size(); ++i) {
        connector.push_back(cellCentre((*path)[i]));
    }

    // Last point (which is the cell containing the chosen node):
    // replace with the exact node position for precision.
    connector.back() = {graph.nodes[best_idx].x, graph.nodes[best_idx].y, 0.0};

    return {best_idx, std::move(connector)};
}

} // namespace finenav

