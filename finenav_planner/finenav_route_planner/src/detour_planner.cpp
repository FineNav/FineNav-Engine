// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "detour_planner.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "astar_algorithm.hpp"
#include "vec3.hpp"

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────

void DetourPlanner::configure(const ConfigType& params)
{
    scan_distance_m_        = params.scan_distance_m;
    grid_resolution_m_      = params.grid_resolution_m;
    blockage_threshold_     = params.blockage_threshold;
    clear_threshold_        = params.clear_threshold;
    goal_tolerance_m_       = params.goal_tolerance_m;
    obstacle_cost_weight_   = params.obstacle_cost_weight;
    max_planning_time_      = params.max_planning_time;
    max_iterations_         = params.max_iterations;
    path_sample_interval_m_ = params.path_sample_interval_m;
}

// ── findBlockage ───────────────────────────────────────────────────────────

std::optional<DetourPlanner::BlockageSegment>
DetourPlanner::findBlockage(const Context& ctx, const IMapView& map_view) const
{
    const auto& poses = ctx.ref_traj.poses;
    if (poses.size() < 2) return std::nullopt;

    const Position3D robot_pos{
        ctx.robot_state.pose.position.x,
        ctx.robot_state.pose.position.y,
        ctx.robot_state.pose.position.z
    };

    // Find the waypoint nearest to the robot as the scan start point.
    size_t scan_start = 0;
    {
        double best_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < poses.size(); ++i) {
            const double dx = poses[i].position.x - robot_pos.x();
            const double dy = poses[i].position.y - robot_pos.y();
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_dist_sq) {
                best_dist_sq = d2;
                scan_start = i;
            }
        }
    }

    std::optional<size_t> blockage_start;
    double accumulated_dist = 0.0;
    size_t last_checked = scan_start;  // track last waypoint we evaluated

    for (size_t i = scan_start; i < poses.size(); ++i) {
        if (i > scan_start) {
            const double dx = poses[i].position.x - poses[i - 1].position.x;
            const double dy = poses[i].position.y - poses[i - 1].position.y;
            accumulated_dist += std::hypot(dx, dy);
        }

        // Sample at path_sample_interval_m_ granularity
        if (i > scan_start && (i + 1 < poses.size()) &&
            (std::fmod(accumulated_dist, path_sample_interval_m_) > path_sample_interval_m_ * 0.5))
            continue;

        last_checked = i;

        const Position3D wp{poses[i].position.x, poses[i].position.y, poses[i].position.z};
        const double cost = map_view.getCost(wp);

        if (!blockage_start.has_value()) {
            if (cost >= blockage_threshold_) {
                blockage_start = i;
            }
        } else {
            // Prefer a real clear point; fall back to scan endpoint otherwise.
            if (cost <= clear_threshold_) {
                return BlockageSegment{*blockage_start, i};
            }
            // Keep scanning until scan_distance_m_ is exhausted
        }

        // Stop scanning beyond scan_distance_m_
        if (accumulated_dist > scan_distance_m_) break;
    }

    // Blockage detected — if no clear point found, fall back to the last
    // waypoint we scanned as the detour goal.
    if (blockage_start.has_value()) {
        size_t exit_idx = std::max(*blockage_start + 1, last_checked);
        exit_idx = std::min(exit_idx, poses.size() - 1);
        return BlockageSegment{*blockage_start, exit_idx};
    }

    return std::nullopt;
}

// ── plan ───────────────────────────────────────────────────────────────────

DetourPlanner::Result DetourPlanner::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    const auto& poses = ctx.ref_traj.poses;
    // No reference trajectory yet (e.g. route_layer has not published).
    // Return empty output to keep the TrackingPolicy loop alive.
    if (poses.empty()) {
        return OutputDataT<DetourPlanner>{};
    }

    // ── 1. Detect blockage ──────────────────────────────────────────────────
    const auto blockage = findBlockage(ctx, map_view);
    if (!blockage.has_value()) {
        {
            static auto last_log = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_log).count() >= 5.0) {
                RCLCPP_INFO(rclcpp::get_logger("DetourPlanner"),
                    "[Detour] Path clear, pass-through. (ref %zu waypoints)", poses.size());
                last_log = now;
            }
        }
        OutputDataT<DetourPlanner> output;
        output.poses = poses;
        return output;
    }

    // Blockage found — always log immediately.
    RCLCPP_INFO(rclcpp::get_logger("DetourPlanner"),
        "[Detour] BLOCKAGE [%zu→%zu] | scanning detour via A*...",
        blockage->start_idx, blockage->exit_idx);

    // ── 2. Grid coordinate helpers ──────────────────────────────────────────
    const double step = map_view.getStepSize();
    if (step <= 0.0) {
        OutputDataT<DetourPlanner> output;
        output.poses = poses;
        return output;
    }

    const double inv_step = 1.0 / step;
    auto worldToCell = [inv_step](double x, double y) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x * inv_step)),
            static_cast<int>(std::floor(y * inv_step)),
            0
        };
    };
    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step, (c.y + 0.5) * step, 0.0 };
    };

    // ── 3. Start cell (robot pose) ─────────────────────────────────────────
    const auto& rp = ctx.robot_state.pose.position;
    const astar::Vec3 start_cell = worldToCell(rp.x, rp.y);

    // ── 4. Goal cell (exit waypoint beyond the blockage) ────────────────────
    const auto& exit_pose = poses[blockage->exit_idx].position;
    const astar::Vec3 goal_cell = worldToCell(exit_pose.x, exit_pose.y);

    // ── 5. A* lambdas ───────────────────────────────────────────────────────
    // Walkability uses the map's own isWalkable() — separate from
    // blockage_threshold_ which only controls findBlockage sensitivity.
    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return c.z == 0
            && map_view.isWalkable(cellCentre(c));
    };

    auto get_cost = [&](const astar::Vec3& c) -> double {
        return map_view.getCost(cellCentre(c));
    };

    // ── 6. Run A* ───────────────────────────────────────────────────────────
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Euclidean);
    algo.setGoalTolerance(goal_tolerance_m_ * inv_step);
    algo.setMaxIterations(max_iterations_);
    algo.setMaxPlanningTime(max_planning_time_);
    algo.setMovementWeight(1.0);
    algo.setGreedyWeight(1.001);
    algo.setObstacleCostWeight(obstacle_cost_weight_);

    auto cell_path = algo.findPath(start_cell, goal_cell);
    if (!cell_path || cell_path->size() < 2) {
        RCLCPP_WARN(rclcpp::get_logger("DetourPlanner"),
            "[Detour] A* FAILED (start/goal unreachable or timeout). "
            "Pass-through. (thr=%.2f goal_tol=%.1fm max_iter=%d max_time=%.2fs)",
            blockage_threshold_, goal_tolerance_m_,
            max_iterations_, max_planning_time_);
        OutputDataT<DetourPlanner> output;
        output.poses = poses;
        return output;
    }

    // ── 7. Stitch: [prefix] + [A* detour] + [suffix] ────────────────────────
    OutputDataT<DetourPlanner> output;
    output.poses.reserve(blockage->start_idx + cell_path->size()
                         + (poses.size() - blockage->exit_idx));

    for (size_t i = 0; i < blockage->start_idx; ++i) {
        output.poses.push_back(poses[i]);
    }

    for (const auto& cell : *cell_path) {
        Position3D wp = cellCentre(cell);
        geometry_msgs::msg::Pose p;
        p.position.x = wp.x();
        p.position.y = wp.y();
        p.position.z = wp.z();
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }

    for (size_t i = blockage->exit_idx; i < poses.size(); ++i) {
        output.poses.push_back(poses[i]);
    }

    RCLCPP_INFO(rclcpp::get_logger("DetourPlanner"),
        "[Detour] A* SUCCESS | detour %zu waypoints | stitched: %zu total.",
        cell_path->size(), output.poses.size());

    return output;
}

} // namespace finenav
