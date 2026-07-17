// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "detour_planner.hpp"

#include <cmath>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include "snap_search_utils.hpp"

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────

void DetourPlanner::configure(const ConfigType& params)
{
    scan_distance_m_      = params.scan_distance_m;
    blockage_threshold_   = params.blockage_threshold;
    reconnect_offset_m_   = params.reconnect_offset_m;
    obstacle_cost_weight_ = params.obstacle_cost_weight;
    max_planning_time_    = params.max_planning_time;
}

// ── findBlockage ───────────────────────────────────────────────────────────

std::optional<size_t> DetourPlanner::findBlockage(
    const Context& ctx, const IMapView& map_view) const
{
    const auto& poses = ctx.ref_traj.poses;
    if (poses.size() < 2) return std::nullopt;

    // Scan starts at the waypoint nearest to the robot.
    size_t scan_start = 0;
    {
        const double rx = ctx.robot_state.pose.position.x;
        const double ry = ctx.robot_state.pose.position.y;
        double best_d2 = std::numeric_limits<double>::max();
        for (size_t i = 0; i < poses.size(); ++i) {
            const double dx = poses[i].position.x - rx;
            const double dy = poses[i].position.y - ry;
            if (dx * dx + dy * dy < best_d2) {
                best_d2    = dx * dx + dy * dy;
                scan_start = i;
            }
        }
    }

    // Walk forward up to scan_distance_m; a waypoint is blocked when it is
    // not walkable OR its (normalised) cost reaches blockage_threshold.
    // Same walkability the detour search uses — one source of truth.
    double acc = 0.0;
    for (size_t i = scan_start; i < poses.size(); ++i) {
        if (i > scan_start) {
            acc += std::hypot(poses[i].position.x - poses[i - 1].position.x,
                              poses[i].position.y - poses[i - 1].position.y);
            if (acc > scan_distance_m_) break;
        }

        const Position3D wp{poses[i].position.x, poses[i].position.y, 0.0};
        if (!map_view.isWalkable(wp) || map_view.getCost(wp) >= blockage_threshold_)
            return i;
    }
    return std::nullopt;
}

// ── plan ───────────────────────────────────────────────────────────────────

DetourPlanner::Result DetourPlanner::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    const auto logger = rclcpp::get_logger("DetourPlanner");

    const auto& poses = ctx.ref_traj.poses;
    // No reference trajectory yet (e.g. route_layer has not published).
    // Return empty output to keep the TrackingPolicy loop alive.
    if (poses.empty()) {
        return OutputDataT<DetourPlanner>{};
    }

    auto passThrough = [&]() {
        OutputDataT<DetourPlanner> output;
        output.poses = poses;
        return output;
    };

    // ── 1. Detect blockage ──────────────────────────────────────────────────
    const auto blocked_idx = findBlockage(ctx, map_view);
    if (!blocked_idx.has_value()) {
        RCLCPP_INFO_THROTTLE(logger, steady_clock, 5000,
            "[Detour] Path clear, pass-through. (ref %zu waypoints)", poses.size());
        return passThrough();
    }

    const double step = map_view.getStepSize();
    if (step <= 0.0) return passThrough();

    // ── 2. Candidate set: waypoints ≥ reconnect_offset_m past the blockage ──
    // Candidates run to the end of the path; ones still inside the obstacle
    // are unreachable and skipped automatically by the search.  If the
    // offset overshoots the path, the last waypoint is the only candidate.
    size_t cand_begin = *blocked_idx;
    {
        double acc = 0.0;
        while (cand_begin + 1 < poses.size() && acc < reconnect_offset_m_) {
            acc += std::hypot(poses[cand_begin + 1].position.x - poses[cand_begin].position.x,
                              poses[cand_begin + 1].position.y - poses[cand_begin].position.y);
            ++cand_begin;
        }
    }

    const snap::CellGrid grid{step};
    const astar::Vec3    origin = grid.toCell(ctx.robot_state.pose.position.x,
                                              ctx.robot_state.pose.position.y);

    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_pts;
    int max_depth = 0;
    for (size_t i = cand_begin; i < poses.size(); ++i) {
        const astar::Vec3 c = grid.toCell(poses[i].position.x, poses[i].position.y);
        cell_pts[c].push_back(i);
        max_depth = std::max(max_depth, snap::chebyshev(c, origin));
    }

    // ── 3. Obstacle-aware search to the nearest reachable candidate ─────────
    auto algo = snap::makeSnapSearch(map_view, grid, origin, max_depth,
                                     obstacle_cost_weight_, max_planning_time_);
    const auto path = algo.findPathToAny(origin,
        [&](const astar::Vec3& c) { return cell_pts.contains(c); });
    if (!path || path->empty()) {
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "[Detour] blockage at waypoint %zu but no reconnect candidate is "
            "reachable — pass-through. (scan=%.1fm offset=%.1fm max_time=%.2fs)",
            *blocked_idx, scan_distance_m_, reconnect_offset_m_, max_planning_time_);
        return passThrough();
    }

    // Earliest waypoint in the goal cell → rejoin the path as early as possible.
    const size_t exit_idx = cell_pts.at(path->back()).front();

    // ── 4. Stitch: [detour: robot → exit waypoint] + [ref suffix] ───────────
    // No prefix: the detour starts at the robot, so prepending the reference
    // path in front of the robot would double back.
    const auto connector = snap::buildConnector(
        ctx.robot_state.pose, *path, grid,
        {poses[exit_idx].position.x, poses[exit_idx].position.y, 0.0});

    OutputDataT<DetourPlanner> output;
    output.poses.reserve(connector.size() + (poses.size() - exit_idx));

    // Drop connector.back(): it coincides with poses[exit_idx], which is
    // appended below with its original orientation intact.
    for (size_t i = 0; i + 1 < connector.size(); ++i) {
        geometry_msgs::msg::Pose p;
        p.position.x    = connector[i].x();
        p.position.y    = connector[i].y();
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }
    for (size_t i = exit_idx; i < poses.size(); ++i) {
        output.poses.push_back(poses[i]);
    }

    RCLCPP_INFO_THROTTLE(logger, steady_clock, 2000,
        "[Detour] blockage at waypoint %zu | detour %zu points → rejoin at "
        "waypoint %zu | %zu total.",
        *blocked_idx, connector.size(), exit_idx, output.poses.size());

    return output;
}

} // namespace finenav
