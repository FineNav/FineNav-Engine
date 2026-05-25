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
    // Grid discretisation
    resolution_ = params.resolution;
    origin_x_   = params.origin_x;
    origin_y_   = params.origin_y;
    origin_z_   = params.origin_z;

    // Dimension
    if (params.dimension == "3D")
        dimension_ = astar::AStarAlgorithm::Dimension::D3;
    else
        dimension_ = astar::AStarAlgorithm::Dimension::D2;

    // Heuristic
    if      (params.heuristic == "Manhattan")  heuristic_ = astar::Heuristic::Manhattan;
    else if (params.heuristic == "Chebyshev")  heuristic_ = astar::Heuristic::Chebyshev;
    else                                        heuristic_ = astar::Heuristic::Euclidean;

    allow_diagonal_    = params.allow_diagonal;
    diagonal_weight_   = params.diagonal_weight;
    goal_tolerance_    = params.goal_tolerance;
    allow_unknown_     = params.allow_unknown;
    max_iterations_    = params.max_iterations;
    max_planning_time_ = params.max_planning_time;
}

// ── plan ───────────────────────────────────────────────────────────────────

AstarPathSearch::Result AstarPathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    // ── helpers: world ↔ cell conversion ──────────────────────────────────
    //
    // These capture resolution_ and origin_* from the planner config.
    // IMapView never needs to know about cells — it only answers
    // world-coordinate queries.

    const double res = resolution_;
    const double ox  = origin_x_, oy = origin_y_, oz = origin_z_;

    auto worldToCell = [&](double x, double y, double z) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor((x - ox) / res)),
            static_cast<int>(std::floor((y - oy) / res)),
            static_cast<int>(std::floor((z - oz) / res))
        };
    };

    // Cell centres are used when querying the map so that the returned
    // world point represents the canonical location of the cell.
    auto cellCentre = [&](const astar::Vec3& c) -> Position3D {
        return {
            ox + (c.x + 0.5) * res,
            oy + (c.y + 0.5) * res,
            oz + (c.z + 0.5) * res
        };
    };

    // ── lambdas injected into AStarAlgorithm ──────────────────────────────
    //
    // AStarAlgorithm only ever calls these — it is completely unaware of
    // IMapView, Position3D, or world coordinates.

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
    algo.setGoalTolerance(goal_tolerance_);
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

