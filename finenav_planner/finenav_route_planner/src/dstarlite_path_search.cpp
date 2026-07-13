// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/dstarlite_path_search.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::OutputTrajectoryT;
namespace finenav {

// ===================================================================
// Configuration
// ===================================================================

void DStarLitePathSearch::configure(const ConfigType& params)
{
    // ── connectivity string → Connectivity ─────────────────
    astar::Connectivity new_conn = astar::Connectivity::N8;
    if      (params.connectivity == "4")  new_conn = astar::Connectivity::N4;
    else if (params.connectivity == "8")  new_conn = astar::Connectivity::N8;
    else if (params.connectivity == "6")  new_conn = astar::Connectivity::N6;
    else if (params.connectivity == "18") new_conn = astar::Connectivity::N18;
    else if (params.connectivity == "26") new_conn = astar::Connectivity::N26;

    // ── heuristic string → Heuristic ───────────────────────
    astar::Heuristic new_heur = astar::Heuristic::Diagonal;
    if      (params.heuristic == "Manhattan") new_heur = astar::Heuristic::Manhattan;
    else if (params.heuristic == "Euclidean") new_heur = astar::Heuristic::Euclidean;
    else if (params.heuristic == "Chebyshev") new_heur = astar::Heuristic::Chebyshev;

    // ── numeric limits ─────────────────────────────────────
    goal_tolerance_m_  = params.goal_tolerance;
    max_expansions_    = params.max_expansions;
    max_path_steps_    = params.max_path_steps;
    max_planning_time_ = params.max_planning_time;
    movement_weight_   = params.movement_weight;
    greedy_weight_     = params.greedy_weight;
    terrain_weight_    = params.terrain_weight;

    // ── structural change detection ────────────────────────
    bool structural_change = false;
    if (engine_) {
        if (new_conn != connectivity_ || new_heur != heuristic_) {
            structural_change = true;
            engine_.reset();
        }
    }

    connectivity_ = new_conn;
    heuristic_    = new_heur;

    // ── push numeric limits & weights to existing engine ────
    if (engine_) {
        engine_->setMaxExpansions(max_expansions_);
        engine_->setMaxPathSteps(max_path_steps_);
        engine_->setMaxPlanningTime(max_planning_time_);
        engine_->setGoalTolerance(
            static_cast<int>(std::ceil(goal_tolerance_m_ / step_size_)));
        engine_->setMovementWeight(movement_weight_);
        engine_->setGreedyWeight(greedy_weight_);
        engine_->setTerrainWeight(terrain_weight_);
    }

    (void)structural_change;
}

void DStarLitePathSearch::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    engine_.reset();
}

// ===================================================================
// Planning
// ===================================================================

DStarLitePathSearch::Result DStarLitePathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const double step = map_view.getStepSize();
    if (step <= 0.0 || !std::isfinite(step))
        return std::nullopt;

    step_size_ = step;

    // ── world → cell conversion (same pattern as AstarPathSearch) ─

    auto worldToCell = [step](double x, double y, double z) -> astar::Vec3 {
        return {static_cast<int>(std::floor(x / step)),
                static_cast<int>(std::floor(y / step)),
                static_cast<int>(std::floor(z / step))};
    };

    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return {(c.x + 0.5) * step,
                (c.y + 0.5) * step,
                (c.z + 0.5) * step};
    };

    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) -> double {
        return map_view.getCost(cellCentre(c));
    };

    const auto& sp = ctx.robot_state.position;
    const auto& gp = ctx.goal_pose.position;

    astar::Vec3 start_cell = worldToCell(sp.x(), sp.y(), sp.z());
    astar::Vec3 goal_cell  = worldToCell(gp.x,  gp.y,  gp.z);

    // ── map dimensions (from extended IMapView) ────────────────

    const int rows   = map_view.rows();
    const int cols   = map_view.cols();
    const int layers = map_view.layers();

    if (rows <= 0 || cols <= 0 || layers <= 0) {
        // Dimensions not provided — cannot proceed
        return std::nullopt;
    }

    // ── engine lifecycle ──────────────────────────────────────

    const bool dimensions_changed =
        !engine_ ||
        engine_->rows()   != rows ||
        engine_->cols()   != cols ||
        engine_->layers() != layers;

    const bool goal_changed =
        !engine_ ||
        engine_->goal() != goal_cell;

    if (dimensions_changed) {
        // Full rebuild: new grid size or first call.

        auto is_walkable = [&](const astar::Vec3& c) -> bool {
            return map_view.isWalkable(cellCentre(c));
        };
        auto get_cost = [&](const astar::Vec3& c) -> double {
            return map_view.getCost(cellCentre(c));
        };

        engine_ = std::make_unique<astar::DStarLiteAlgorithm>(
            rows, cols, is_walkable, get_cost,
            connectivity_, heuristic_, layers);

        engine_->setMaxExpansions(max_expansions_);
        engine_->setMaxPathSteps(max_path_steps_);
        engine_->setMaxPlanningTime(max_planning_time_);
        engine_->setGoalTolerance(
            static_cast<int>(std::ceil(goal_tolerance_m_ / step_size_)));
        engine_->setMovementWeight(movement_weight_);
        engine_->setGreedyWeight(greedy_weight_);
        engine_->setTerrainWeight(terrain_weight_);

        engine_->initialize(start_cell, goal_cell);
        engine_->computeShortestPath();
    }
    else {
        // Incremental replan: sync map changes, then replan.
        engine_->setGoalTolerance(
            static_cast<int>(std::ceil(goal_tolerance_m_ / step_size_)));

        if (goal_changed) {
            // Goal changed — full reinitialize needed.
            engine_->initialize(start_cell, goal_cell);
            engine_->computeShortestPath();
        } else {
            // Same goal, possibly new obstacles — incremental replan.
            engine_->syncMap();
            engine_->replan(start_cell);
        }
    }

    // ── extract path and convert to world coordinates ─────────

    auto cell_path = engine_->extractPath();
    if (cell_path.empty())
        return std::nullopt;

    OutputTrajectoryT<DStarLitePathSearch> output;
    output.poses.reserve(cell_path.size());

    for (const auto& cell : cell_path) {
        const Position3D w = cellCentre(cell);
        geometry_msgs::msg::Pose pose;
        pose.position.x    = w.x();
        pose.position.y    = w.y();
        pose.position.z    = w.z();
        pose.orientation.w = 1.0;
        output.poses.push_back(pose);
    }

    return output;
}

}  // namespace finenav
