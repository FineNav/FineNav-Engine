// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <cmath>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/dstarlite_params.hpp"

#include "finenav_route_planner/dstarlite_algorithm.hpp"
#include "finenav_route_planner/imap_view.hpp"
#include "finenav_route_planner/vec3.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

// =============================================================
// DStarLitePathSearch
//
// FineNav planner that performs incremental D* Lite path search.
//
// Compared to A*, D* Lite is stateful: it maintains g/rhs values
// and a priority queue across planning calls.  When the map
// changes (new obstacles discovered), only the affected portion
// of the state is recomputed — replanning is O(affected area)
// rather than O(grid).
//
// The planner defines its own IMapView that extends astar::IMapView
// with grid-dimension queries (rows/cols/layers) required by D* Lite's
// pre-allocated state arrays.
// =============================================================

class DStarLitePathSearch {
public:
    DStarLitePathSearch() = default;

    // ── FineNav framework metadata ────────────────────────────

    using ConfigType = dstarlite_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = false,
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

    // ── D* Lite-specific IMapView ──────────────────────────────
    //
    // Extends the standard astar::IMapView (isWalkable, getCost,
    // getStepSize) with grid-dimension queries needed by D* Lite's
    // pre-allocated flat arrays.
    //
    // Existing map adapters for A*/BFS/Route are unaffected —
    // they continue to implement astar::IMapView directly.
    // Only maps used with DStarLitePathSearch need to provide
    // rows()/cols()/layers().

    class IMapView : public astar::IMapView {
    public:
        /// Number of rows (y-axis) in the grid.
        virtual int rows() const = 0;

        /// Number of columns (x-axis) in the grid.
        virtual int cols() const = 0;

        /// Number of layers (z-axis).  Default 1 = 2-D grid.
        virtual int layers() const { return 1; }
    };

    using Context = PlanningContext<DStarLitePathSearch>;
    using Result  = std::optional<OutputTrajectoryT<DStarLitePathSearch>>;

    // ── PlannerConcept interface ───────────────────────────────

    /// Configure from YAML-generated parameters.
    /// Structural changes (connectivity/heuristic) trigger engine rebuild.
    void configure(const ConfigType& params);

    /// Reset engine state at episode boundary (new goal).
    void reset();

    /// Plan a path from current robot pose to goal.
    /// On first call: builds engine + initialize + computeShortestPath.
    /// On subsequent calls: syncMap + replan (incremental).
    Result plan(const Context& ctx, const IMapView& map_view);

private:
    // ── settings ──────────────────────────────────────────────

    astar::Connectivity connectivity_      = astar::Connectivity::N8;
    astar::Heuristic    heuristic_         = astar::Heuristic::Diagonal;
    double goal_tolerance_m_  = 0.5;
    int    max_expansions_    = 1'000'000;
    int    max_path_steps_    = 10'000;
    double max_planning_time_ = 5.0;
    double movement_weight_   = 1.0;
    double greedy_weight_     = 1.0;
    double terrain_weight_    = 1.0;

    // ── persistent state ──────────────────────────────────────

    std::unique_ptr<astar::DStarLiteAlgorithm> engine_;
    double       step_size_    = 1.0;

    // ── thread safety ────────────────────────────────────────

    mutable std::mutex mutex_;
};

}  // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::DStarLitePathSearch, dstarlite_planner)
