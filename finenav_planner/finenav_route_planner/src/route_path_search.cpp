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

// ── grid helpers (shared by both snap searches) ────────────────────────────

namespace {

/// World ↔ grid-cell conversion at the map resolution.
struct CellGrid {
    double step;

    astar::Vec3 toCell(double x, double y) const {
        return {static_cast<int>(std::floor(x / step)),
                static_cast<int>(std::floor(y / step)), 0};
    }
    Position3D centre(const astar::Vec3& c) const {
        return {(c.x + 0.5) * step, (c.y + 0.5) * step, 0.0};
    }
};

int chebyshev(const astar::Vec3& a, const astar::Vec3& b)
{
    return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

/// Dijkstra (h=0) engine on the real occupancy map, bounded to a box of
/// `max_depth` cells around `origin` so an unreachable goal set cannot
/// trigger an unbounded expansion.
astar::AStarAlgorithm makeSnapSearch(
    const astar::IMapView& map_view,
    const CellGrid&        grid,
    const astar::Vec3&     origin,
    int                    max_depth,
    double                 obstacle_cost_weight)
{
    auto is_walkable = [mv = &map_view, grid, origin, max_depth](const astar::Vec3& c) {
        return c.z == 0 &&
               chebyshev(c, origin) <= max_depth &&
               mv->isWalkable(grid.centre(c));
    };
    auto get_cost = [mv = &map_view, grid](const astar::Vec3& c) {
        return mv->getCost(grid.centre(c));
    };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(5.0);  // TODO: 后面这些应该也成为动态参数
    algo.setObstacleCostWeight(obstacle_cost_weight);
    return algo;
}

/// Connector = exact start pose, then the grid path's cell centres
/// (cells[0] is the cell containing the pose and is skipped), then the
/// exact snap point.
std::vector<Position3D> buildConnector(
    const geometry_msgs::msg::Pose& pose,
    const std::vector<astar::Vec3>& cells,
    const CellGrid&                 grid,
    const Position3D&               end)
{
    std::vector<Position3D> conn;
    conn.reserve(cells.size() + 1);
    conn.push_back({pose.position.x, pose.position.y, 0.0});
    for (size_t i = 1; i < cells.size(); ++i)
        conn.push_back(grid.centre(cells[i]));
    if (conn.size() > 1)
        conn.back() = end;
    else
        conn.push_back(end);
    return conn;
}

double polylineLength(const std::vector<std::pair<double, double>>& pl)
{
    double len = 0.0;
    for (size_t i = 1; i < pl.size(); ++i)
        len += std::hypot(pl[i].first - pl[i - 1].first,
                          pl[i].second - pl[i - 1].second);
    return len;
}

/// Distance from point p to segment (a, b).
double pointSegmentDist(const Position3D&               p,
                        const std::pair<double, double>& a,
                        const std::pair<double, double>& b)
{
    const double vx = b.first - a.first, vy = b.second - a.second;
    const double wx = p.x() - a.first,   wy = p.y() - a.second;
    const double t  = std::clamp((wx * vx + wy * vy) / (vx * vx + vy * vy + 1e-12), 0.0, 1.0);
    return std::hypot(wx - t * vx, wy - t * vy);
}

}  // anonymous namespace

// ── plan ───────────────────────────────────────────────────────────────────

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    std::lock_guard<std::mutex> lock(*graph_mutex_);

    if (!cached_graph_ || cached_graph_->nodes.empty()) return std::nullopt;
    const auto& graph = *cached_graph_;

    // ── 1. Snap robot & goal to the nearest reachable graph node ─────────
    NodeSnap start_snap = snapToNearestNode(graph, ctx.robot_state.pose, map_view);
    if (!start_snap.valid()) return std::nullopt;

    const NodeSnap goal_snap = snapToNearestNode(graph, ctx.goal_pose, map_view);
    if (!goal_snap.valid()) return std::nullopt;

    // ── 2. Dijkstra on the route graph ────────────────────────────────────
    std::vector<size_t> node_seq;
    auto route_opt = route::findRoute(
        graph, start_snap.node_index, goal_snap.node_index, path_density_, &node_seq);
    if (!route_opt || route_opt->empty()) return std::nullopt;

    // ── 3. Start prune ────────────────────────────────────────────────────
    // TODO: goal-side prune — symmetric treatment of the goal connector is
    //       still missing; the path may backtrack near a snapped goal node.
    //
    // If the robot projects positively onto the first route edge
    // (node_seq[0] → node_seq[1]) it is already past the first node, and
    // entering the graph there would backtrack.  Snap onto the edge itself
    // (virtual node + edge split on a graph copy) and re-route from there.
    std::vector<Position3D> start_connector = std::move(start_snap.connector_path);
    if (node_seq.size() >= 2) {
        const auto& fn = graph.nodes[node_seq[0]];
        const auto& nn = graph.nodes[node_seq[1]];
        const double dot = (nn.x - fn.x) * (ctx.robot_state.pose.position.x - fn.x)
                         + (nn.y - fn.y) * (ctx.robot_state.pose.position.y - fn.y);
        if (dot > 0.0) {
            EdgeSnap edge_snap = snapToEdge(
                graph, ctx.robot_state.pose, map_view, node_seq[0], node_seq[1]);
            if (edge_snap.valid()) {
                route::Graph pruned = graph;   // deep copy, only when the prune fires
                const size_t vi = insertSnapNode(pruned, edge_snap);
                auto rerouted = route::findRoute(
                    pruned, vi, goal_snap.node_index, path_density_);
                // On failure keep the original (backtracking but valid) route.
                if (rerouted && !rerouted->empty()) {
                    route_opt       = std::move(rerouted);
                    start_connector = std::move(edge_snap.connector_path);
                }
            }
        }
    }

    // ── 4. Assemble: start connector + route + reversed goal connector ───
    OutputDataT<RoutePathSearch> output;
    output.poses.reserve(start_connector.size()
                       + route_opt->size()
                       + goal_snap.connector_path.size());

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
    for (const auto& [x, y] : *route_opt)
        appendPose(x, y);
    // Goal connector was built goal_pose → snap point; walk it backwards.
    for (auto it = goal_snap.connector_path.rbegin();
         it != goal_snap.connector_path.rend(); ++it)
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
    const CellGrid    grid{map_view.getStepSize()};
    const astar::Vec3 origin = grid.toCell(pose.position.x, pose.position.y);

    // cell → node indices; max_depth bounds the search box.
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_nodes;
    int max_depth = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const astar::Vec3 c = grid.toCell(graph.nodes[i].x, graph.nodes[i].y);
        cell_nodes[c].push_back(i);
        max_depth = std::max(max_depth, chebyshev(c, origin));
    }

    auto algo = makeSnapSearch(map_view, grid, origin, max_depth, obstacle_cost_weight_);
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

    snap.connector_path = buildConnector(pose, *path, grid,
        {graph.nodes[snap.node_index].x, graph.nodes[snap.node_index].y, 0.0});
    return snap;
}

// ── snapToEdge ──────────────────────────────────────────────────────────────

RoutePathSearch::EdgeSnap RoutePathSearch::snapToEdge(
    const route::Graph&             graph,
    const geometry_msgs::msg::Pose& pose,
    const IMapView&                 map_view,
    size_t                          edge_from,
    size_t                          edge_to) const
{
    const CellGrid    grid{map_view.getStepSize()};
    const astar::Vec3 origin = grid.toCell(pose.position.x, pose.position.y);

    // Collect the intermediate polyline points of edge(edge_from → edge_to).
    // Endpoints coincide with the nodes themselves and are skipped.
    struct PtRef { size_t ei, pi; };
    std::unordered_map<astar::Vec3, std::vector<PtRef>, astar::Vec3Hash> cell_pts;
    size_t first_ei  = SIZE_MAX;
    int    max_depth = 0;
    for (size_t ei = 0; ei < graph.adj[edge_from].size(); ++ei) {
        if (graph.adj[edge_from][ei].to_index != edge_to) continue;
        if (first_ei == SIZE_MAX) first_ei = ei;
        const auto& pl = graph.adj[edge_from][ei].polyline;
        for (size_t pi = 1; pi + 1 < pl.size(); ++pi) {
            const astar::Vec3 c = grid.toCell(pl[pi].first, pl[pi].second);
            cell_pts[c].push_back({ei, pi});
            max_depth = std::max(max_depth, chebyshev(c, origin));
        }
    }
    if (first_ei == SIZE_MAX) return {};   // no such edge — should not happen

    EdgeSnap snap;

    if (cell_pts.empty()) {
        // ── Projection fallback: the edge has no intermediate points ──────
        // Project the pose onto the edge chord; splitDirectedEdge later
        // inserts the projected point into the polyline before splitting.
        const auto&  fn = graph.nodes[edge_from];
        const auto&  nn = graph.nodes[edge_to];
        const double vx = nn.x - fn.x, vy = nn.y - fn.y;
        const double t  = std::clamp(
            ((pose.position.x - fn.x) * vx + (pose.position.y - fn.y) * vy) /
                (vx * vx + vy * vy + 1e-12),
            0.0, 1.0);
        const Position3D proj{fn.x + t * vx, fn.y + t * vy, 0.0};

        auto algo = makeSnapSearch(map_view, grid, origin,
                                   std::numeric_limits<int>::max(), obstacle_cost_weight_);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        const auto path = algo.findPath(origin, grid.toCell(proj.x(), proj.y()));
        if (!path || path->empty()) return {};

        snap.edge_from      = edge_from;
        snap.edge_idx       = first_ei;
        snap.pt_idx         = SIZE_MAX;   // not a polyline vertex
        snap.snap_point     = proj;
        snap.connector_path = buildConnector(pose, *path, grid, proj);
        return snap;
    }

    // ── Obstacle-aware search to the nearest reachable point on the edge ──
    auto algo = makeSnapSearch(map_view, grid, origin, max_depth, obstacle_cost_weight_);
    const auto path = algo.findPathToAny(origin,
        [&](const astar::Vec3& c) { return cell_pts.contains(c); });
    if (!path || path->empty()) return {};

    // Several points may share the goal cell — pick the closest to the pose.
    double best = std::numeric_limits<double>::infinity();
    for (const auto& [ei, pi] : cell_pts.at(path->back())) {
        const auto& pt = graph.adj[edge_from][ei].polyline[pi];
        const double d = std::hypot(pt.first - pose.position.x,
                                    pt.second - pose.position.y);
        if (d < best) {
            best            = d;
            snap.edge_from  = edge_from;
            snap.edge_idx   = ei;
            snap.pt_idx     = pi;
            snap.snap_point = {pt.first, pt.second, 0.0};
        }
    }
    snap.connector_path = buildConnector(pose, *path, grid, snap.snap_point);
    return snap;
}

// ── insertSnapNode ──────────────────────────────────────────────────────────

size_t RoutePathSearch::insertSnapNode(
    route::Graph&   graph,
    const EdgeSnap& snap) const
{
    // Virtual node: no id, not registered in id_index.
    const size_t vi = graph.nodes.size();
    graph.nodes.push_back({/*id=*/0, snap.snap_point.x(), snap.snap_point.y(), /*frame=*/""});
    graph.adj.resize(graph.nodes.size());

    const size_t to = graph.adj[snap.edge_from][snap.edge_idx].to_index;
    splitDirectedEdge(graph, snap.edge_from, snap.edge_idx, snap.snap_point, vi);

    // Split the reverse edge (to → edge_from) too, if one exists.
    for (size_t ei = 0; ei < graph.adj[to].size(); ++ei) {
        if (graph.adj[to][ei].to_index == snap.edge_from) {
            splitDirectedEdge(graph, to, ei, snap.snap_point, vi);
            break;
        }
    }
    return vi;
}

// ── splitDirectedEdge ───────────────────────────────────────────────────────
//
// Replaces graph.adj[from][edge_idx] (from → dst) with two sub-edges:
//   from → virtual_idx   (polyline up to and including the split point)
//   virtual_idx → dst    (polyline from the split point onward)
// The edge cost is pro-rated by sub-polyline length so that fixed-cost
// overrides keep their total weight.

void RoutePathSearch::splitDirectedEdge(
    route::Graph&     graph,
    size_t            from,
    size_t            edge_idx,
    const Position3D& p,
    size_t            virtual_idx) const
{
    auto& nb = graph.adj[from][edge_idx];
    auto& pl = nb.polyline;
    if (pl.size() < 2) return;

    constexpr double kTol = 1e-6;

    // Locate the split index: an existing vertex matching p, otherwise
    // insert p into the segment it lies closest to (projection fallback).
    size_t split_idx = SIZE_MAX;
    for (size_t i = 0; i < pl.size(); ++i) {
        if (std::hypot(pl[i].first - p.x(), pl[i].second - p.y()) <= kTol) {
            split_idx = i;
            break;
        }
    }
    if (split_idx == SIZE_MAX) {
        size_t best_seg = 0;
        double best_d   = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i + 1 < pl.size(); ++i) {
            const double d = pointSegmentDist(p, pl[i], pl[i + 1]);
            if (d < best_d) {
                best_d   = d;
                best_seg = i;
            }
        }
        pl.insert(pl.begin() + best_seg + 1, {p.x(), p.y()});
        split_idx = best_seg + 1;
    }

    std::vector<std::pair<double, double>> pl_first(pl.begin(), pl.begin() + split_idx + 1);
    std::vector<std::pair<double, double>> pl_second(pl.begin() + split_idx, pl.end());

    const double len_first  = polylineLength(pl_first);
    const double len_second = polylineLength(pl_second);
    const double total      = len_first + len_second;
    const double cost_first  = (total > 1e-9) ? nb.cost * (len_first / total)  : nb.cost * 0.5;
    const double cost_second = (total > 1e-9) ? nb.cost * (len_second / total) : nb.cost * 0.5;

    const size_t dst = nb.to_index;
    nb.to_index = virtual_idx;
    nb.polyline = std::move(pl_first);
    nb.cost     = cost_first;

    graph.adj[virtual_idx].push_back({dst, cost_second, std::move(pl_second)});
}

} // namespace finenav
