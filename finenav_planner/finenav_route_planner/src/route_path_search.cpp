// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route_path_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_map>

#include <geometry_msgs/msg/pose.hpp>

#include "snap_search_utils.hpp"

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────

void RoutePathSearch::configure(const ConfigType& params)
{
    path_density_         = params.path_density;
    obstacle_cost_weight_ = params.obstacle_cost_weight;
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

    // Throttled failure logs: plan() may be called in a loop (eval node,
    // replanning), so each failure reason is reported at most every 2 s.
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    const auto logger = rclcpp::get_logger("RoutePathSearch");

    if (!cached_graph_ || cached_graph_->nodes.empty()) {
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "plan failed: no route graph loaded (waiting for /geojson).");
        return std::nullopt;
    }
    const auto& graph = *cached_graph_;

    // ── 1. Snap robot & goal to the nearest reachable graph node ─────────
    NodeSnap start_snap = snapToNearestNode(graph, ctx.robot_state.pose, map_view);
    if (!start_snap.valid()) {
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "plan failed: cannot snap start (%.2f, %.2f) to any graph node — "
            "start cell blocked, or every node is unreachable on the map.",
            ctx.robot_state.pose.position.x, ctx.robot_state.pose.position.y);
        return std::nullopt;
    }

    NodeSnap goal_snap = snapToNearestNode(graph, ctx.goal_pose, map_view);
    if (!goal_snap.valid()) {
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "plan failed: cannot snap goal (%.2f, %.2f) to any graph node — "
            "goal cell blocked, or every node is unreachable on the map.",
            ctx.goal_pose.position.x, ctx.goal_pose.position.y);
        return std::nullopt;
    }

    // ── 2. Dijkstra on the route graph ────────────────────────────────────
    const auto route_opt = route::findRoute(
        graph, start_snap.node_index, goal_snap.node_index, path_density_);
    if (!route_opt || route_opt->empty()) {
        // Snapping is purely nearest-by-distance; it does NOT check graph
        // connectivity.  The route graph is required to be connected — a
        // failure here almost always means the GeoJSON contains isolated
        // components or one-way-only chains.
        const auto& sn = graph.nodes[start_snap.node_index];
        const auto& gn = graph.nodes[goal_snap.node_index];
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "plan failed: no route from node id=%u (%.2f, %.2f) to node id=%u "
            "(%.2f, %.2f) — the graph is disconnected (isolated component or "
            "one-way edge chain in the GeoJSON).",
            sn.id, sn.x, sn.y, gn.id, gn.x, gn.y);
        return std::nullopt;
    }
    const route::RoutePoses& route = *route_opt;

    // ── 3. Trim the route curve at both ends ──────────────────────────────
    // Entering/leaving the route at node centres backtracks whenever the
    // robot (or the goal) is already alongside the route.  Instead, project
    // both onto the route curve — obstacle-aware BFS to the nearest
    // reachable route point — and keep only the segment in between.  A
    // single global nearest-point query handles being past one or MANY
    // nodes, with no iteration or termination heuristics.
    //
    // The exit search is restricted to the tail after the entry point, so
    // entry ≤ exit always holds.  On (unlikely) trim failure the node
    // connectors from step 1 keep the plan valid.
    //
    // Note: with path_density <= 0 the route contains node centres only, so
    // trimming degrades to node granularity.
    size_t begin_idx = 0;
    size_t end_idx   = route.size();
    std::vector<Position3D> start_connector = std::move(start_snap.connector_path);
    std::vector<Position3D> goal_connector  = std::move(goal_snap.connector_path);

    RouteSnap entry = snapToRoute(route, 0, ctx.robot_state.pose, map_view);
    if (entry.valid()) {
        begin_idx       = entry.index;
        start_connector = std::move(entry.connector_path);
    }
    RouteSnap exit = snapToRoute(route, begin_idx, ctx.goal_pose, map_view);
    if (exit.valid()) {
        end_idx        = exit.index + 1;
        goal_connector = std::move(exit.connector_path);
    }

    // ── 4. Assemble: start connector + route segment + reversed goal conn ─
    OutputDataT<RoutePathSearch> output;
    output.poses.reserve(start_connector.size()
                       + (end_idx - begin_idx)
                       + goal_connector.size());

    // Consecutive duplicate points (connector/route boundaries) are dropped.
    auto appendPose = [&](double x, double y) {
        if (!output.poses.empty() &&
            std::hypot(output.poses.back().position.x - x,
                       output.poses.back().position.y - y) < 1e-6)
            return;
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    };

    for (const auto& pt : start_connector)
        appendPose(pt.x(), pt.y());
    for (size_t i = begin_idx; i < end_idx; ++i)
        appendPose(route[i].first, route[i].second);
    // Goal connector was built goal_pose → route point; walk it backwards.
    for (auto it = goal_connector.rbegin(); it != goal_connector.rend(); ++it)
        appendPose(it->x(), it->y());

    return output;
}

// ── snapToNearestNode ───────────────────────────────────────────────────────
//
// Runs AStarAlgorithm::findPathToAny (h=0, Dijkstra) on the real occupancy
// map.  The search expands from the cell containing `pose` outward until it
// finds a cell that contains at least one route graph node.
//
// Using the real map means the connector is obstacle-aware: a node that can
// only be reached by going through a wall will not be selected.

RoutePathSearch::NodeSnap RoutePathSearch::snapToNearestNode(
    const route::Graph&             graph,
    const geometry_msgs::msg::Pose& pose,
    const IMapView&                 map_view) const
{
    const snap::CellGrid grid{map_view.getStepSize()};
    const astar::Vec3 origin = grid.toCell(pose.position.x, pose.position.y);

    // cell → node indices; max_depth bounds the search box.
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_nodes;
    int max_depth = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const astar::Vec3 c = grid.toCell(graph.nodes[i].x, graph.nodes[i].y);
        cell_nodes[c].push_back(i);
        max_depth = std::max(max_depth, snap::chebyshev(c, origin));
    }

    auto algo = snap::makeSnapSearch(map_view, grid, origin, max_depth, obstacle_cost_weight_);
    const auto path = algo.findPathToAny(origin,
        [&](const astar::Vec3& c) { return cell_nodes.contains(c); });
    if (!path || path->empty()) return {};

    // Several nodes may share the goal cell — pick the closest to the pose.
    NodeSnap snap;
    double best = std::numeric_limits<double>::infinity();
    for (const size_t idx : cell_nodes.at(path->back())) {
        const double d = std::hypot(graph.nodes[idx].x - pose.position.x,
                                    graph.nodes[idx].y - pose.position.y);
        if (d < best) {
            best            = d;
            snap.node_index = idx;
        }
    }

    snap.connector_path = snap::buildConnector(pose, *path, grid,
        {graph.nodes[snap.node_index].x, graph.nodes[snap.node_index].y, 0.0});
    return snap;
}

// ── snapToRoute ─────────────────────────────────────────────────────────────
//
// Same obstacle-aware search, but the goal set is the points of
// route[begin_idx..end).  Used by plan() to trim the route curve: the
// nearest reachable route point is where the robot enters (begin_idx = 0)
// or where the path leaves toward the goal (begin_idx = entry index).

RoutePathSearch::RouteSnap RoutePathSearch::snapToRoute(
    const route::RoutePoses&        route,
    size_t                          begin_idx,
    const geometry_msgs::msg::Pose& pose,
    const IMapView&                 map_view) const
{
    const snap::CellGrid grid{map_view.getStepSize()};
    const astar::Vec3 origin = grid.toCell(pose.position.x, pose.position.y);

    // cell → route point indices; max_depth bounds the search box.
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_pts;
    int max_depth = 0;
    for (size_t i = begin_idx; i < route.size(); ++i) {
        const astar::Vec3 c = grid.toCell(route[i].first, route[i].second);
        cell_pts[c].push_back(i);
        max_depth = std::max(max_depth, snap::chebyshev(c, origin));
    }
    if (cell_pts.empty()) return {};

    auto algo = snap::makeSnapSearch(map_view, grid, origin, max_depth, obstacle_cost_weight_);
    const auto path = algo.findPathToAny(origin,
        [&](const astar::Vec3& c) { return cell_pts.contains(c); });
    if (!path || path->empty()) return {};

    // Several route points may share the goal cell — pick the closest to
    // the pose (ties keep the earliest index, i.e. the least route is cut).
    RouteSnap snap;
    double best = std::numeric_limits<double>::infinity();
    for (const size_t idx : cell_pts.at(path->back())) {
        const double d = std::hypot(route[idx].first - pose.position.x,
                                    route[idx].second - pose.position.y);
        if (d < best) {
            best       = d;
            snap.index = idx;
        }
    }

    snap.connector_path = snap::buildConnector(pose, *path, grid,
        {route[snap.index].first, route[snap.index].second, 0.0});
    return snap;
}

} // namespace finenav
