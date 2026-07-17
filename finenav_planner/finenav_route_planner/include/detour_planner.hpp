// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_route_planner/detour_params.hpp"

#include "imap_view.hpp"   // astar::IMapView — shared interface

namespace finenav {

// =============================================================
// DetourPlanner
//
// Monitors a reference trajectory for blocked segments and
// generates A*-based detour paths around dynamic obstacles.
//
// Designed to sit between a global route planner and MPPI in
// the FineNav pipeline.  When no blockage is detected the
// reference trajectory is passed through unchanged.
//
// IMapView cost contract:
//   getCost() MUST return normalised costs in [0.0, 1.0]
//   (e.g. raw TerrainAnalyzer cost / 254.0).
//   isWalkable() returns false for lethal obstacles.
// =============================================================

class DetourPlanner {
public:
    DetourPlanner() = default;

    // ── FineNav framework metadata ────────────────────────────

    using ConfigType = detour_planner::Params;

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

    using IMapView = astar::IMapView;

    using Context = PlanningContext<DetourPlanner>;
    using Result  = std::optional<OutputDataT<DetourPlanner>>;

    void configure(const ConfigType& params);
    void reset() {}

    /// Main entry point.
    /// Returns a detour-modified trajectory if the reference path is blocked,
    /// or the original reference trajectory if the path is clear.
    Result plan(const Context& ctx, const IMapView& map_view);

private:
    struct BlockageSegment {
        size_t start_idx;
        size_t exit_idx;
    };

    /// Walk along ctx.ref_traj and locate the first blocked segment.
    std::optional<BlockageSegment> findBlockage(
        const Context& ctx, const IMapView& map_view) const;

    // ── parameters ────────────────────────────────────────────
    double scan_distance_m_        = 5.0;   // forward scan for blockage detection
    double grid_resolution_m_      = 0.1;
    double blockage_threshold_     = 0.8;
    double clear_threshold_        = 0.2;
    double goal_tolerance_m_       = 1.0;   // A* goal tolerance (metres)
    double obstacle_cost_weight_   = 1.0;
    double max_planning_time_      = 0.1;
    int    max_iterations_         = 10000;
    double path_sample_interval_m_ = 0.1;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::DetourPlanner, detour_planner)
