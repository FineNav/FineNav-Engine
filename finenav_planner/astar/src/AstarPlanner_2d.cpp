// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "AstarPlanner_2d.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────

void AstarPathSearch::configure(const ConfigType& params)
{
    if (params.dimension == "3D")
        dimension_ = astar::AStarAlgorithm::Dimension::D3;
    else
        dimension_ = astar::AStarAlgorithm::Dimension::D2;

    if      (params.heuristic == "Manhattan")  heuristic_ = astar::Heuristic::Manhattan;
    else if (params.heuristic == "Chebyshev")  heuristic_ = astar::Heuristic::Chebyshev;
    else                                        heuristic_ = astar::Heuristic::Euclidean;

    allow_diagonal_    = params.allow_diagonal;
    diagonal_weight_   = params.diagonal_weight;
    goal_tolerance_m_  = params.goal_tolerance;   // stored in metres
    allow_unknown_     = params.allow_unknown;
    max_iterations_    = params.max_iterations;
    max_planning_time_ = params.max_planning_time;
}

// ── plan ───────────────────────────────────────────────────────────────────

AstarPathSearch::Result AstarPathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    // ── read step size from the map (single source of truth) ──────────────
    const double step = map_view.getStepSize();

    // ── world ↔ cell helpers ───────────────────────────────────────────────
    //
    // Origin is always (0,0,0) in FineNav (WORLD_FIXED_FRAME).
    // Cell centre = (ix + 0.5) * step  for each axis.

    auto worldToCell = [step](double x, double y, double z) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            static_cast<int>(std::floor(z / step))
        };
    };

    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step,
                 (c.y + 0.5) * step,
                 (c.z + 0.5) * step };
    };

    // ── lambdas injected into AStarAlgorithm ──────────────────────────────

    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) -> double {
        return map_view.getCost(cellCentre(c));
    };
    auto is_unknown = [&](const astar::Vec3& c) -> bool {
        return map_view.isUnknown(cellCentre(c));
    };

    // ── run A* in grid space ───────────────────────────────────────────────

    const auto& sp = ctx.start_pose.position;
    const auto& gp = ctx.goal_pose.position;

    astar::Vec3 start = worldToCell(sp.x, sp.y, sp.z);
    astar::Vec3 goal  = worldToCell(gp.x, gp.y, gp.z);

    astar::AStarAlgorithm algo(is_walkable, get_cost, is_unknown);
    algo.setDimension(dimension_);
    algo.setHeuristic(heuristic_);
    algo.setAllowDiagonal(allow_diagonal_);
    algo.setDiagonalWeight(diagonal_weight_);
    algo.setGoalTolerance(goal_tolerance_m_ / step);  // metres → cells
    algo.setAllowUnknown(allow_unknown_);
    algo.setMaxIterations(max_iterations_);
    algo.setMaxPlanningTime(max_planning_time_);

    auto cell_path = algo.findPath(start, goal);
    if (!cell_path)
        return std::nullopt;

    // ── convert cell path → world poses ───────────────────────────────────

    OutputDataT<AstarPathSearch> output;
    output.poses.reserve(cell_path->size());

    for (const auto& cell : *cell_path)
    {
        Position3D w = cellCentre(cell);

        geometry_msgs::msg::Pose p;
        p.position.x    = w.x();
        p.position.y    = w.y();
        p.position.z    = w.z();
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }

    return output;
}

} // namespace finenav

