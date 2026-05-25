// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <cmath>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_planner_astar/astar_params.hpp"

#include "astar_algorithm.hpp"

namespace finenav {

// =============================================================
// AstarPathSearch
//
// FineNav planner that performs uniform-grid A* path search.
//
// The planner owns the grid discretisation parameters
// (resolution, origin).  The user only needs to implement the
// nested IMapView interface using their own world-coordinate
// map data — they do not need to know anything about Vec3,
// grid cells, or the internal A* algorithm.
//
// Usage:
//   class MyMapView : public AstarPathSearch::IMapView {
//       bool   isWalkable(const Position3D& p) const override { ... }
//       double getCost   (const Position3D& p) const override { ... }
//   };
//   planner.plan(ctx, MyMapView{...});
// =============================================================

class AstarPathSearch {
public:
    AstarPathSearch() = default;

    // ── FineNav framework metadata ────────────────────────────

    using ConfigType = astar_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };

    // ── User-facing map interface ─────────────────────────────
    //
    // All coordinates are world-space (metres, same frame as
    // the start / goal poses in the planning context).
    // The user implements this and passes it to plan().

    class IMapView {
    public:
        virtual ~IMapView() = default;

        /// Can the robot pass through world point p?
        virtual bool isWalkable(const Position3D& p) const = 0;

        /// Extra traversal cost at p.  Must be >= 0.
        /// A* edge weight = step_cost * (1 + getCost(p)).
        /// Return 0.0 for a uniform-cost map.
        virtual double getCost(const Position3D& p) const = 0;

        /// Is the occupancy at p unknown?  Default: no.
        virtual bool isUnknown(const Position3D& p) const { return false; }
    };

    // ── FineNav planner interface ─────────────────────────────

    using Context = PlanningContext<AstarPathSearch>;
    using Result  = std::optional<OutputDataT<AstarPathSearch>>;

    void configure(const ConfigType& params);

    Result plan(const Context& ctx, const IMapView& map_view);

private:

    // Grid discretisation (owned by this planner)
    double resolution_ = 0.05;   // metres per cell
    double origin_x_   = 0.0;
    double origin_y_   = 0.0;
    double origin_z_   = 0.0;

    // A* search parameters
    astar::AStarAlgorithm::Dimension dimension_     = astar::AStarAlgorithm::Dimension::D2;
    astar::Heuristic                 heuristic_     = astar::Heuristic::Euclidean;
    bool   allow_diagonal_    = true;
    double diagonal_weight_   = std::sqrt(2.0);
    double goal_tolerance_    = 0.25;
    bool   allow_unknown_     = true;
    int    max_iterations_    = 1000000;
    double max_planning_time_ = 5.0;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::AstarPathSearch, astar_planner)

