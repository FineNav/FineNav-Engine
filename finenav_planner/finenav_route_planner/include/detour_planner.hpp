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
// generates grid-search detour paths around dynamic obstacles.
//
// Designed to sit between a global route planner and MPPI in
// the FineNav pipeline.  When no blockage is detected the
// reference trajectory is passed through unchanged.
//
// Flow in plan():
//   1. Scan the reference path forward from the waypoint nearest
//      to the robot (up to scan_distance_m).  A waypoint is
//      "blocked" when !isWalkable OR cost >= blockage_threshold.
//   2. No blocked waypoint → pass the reference through.
//   3. Candidate set = all waypoints at least reconnect_offset_m
//      past the blocked waypoint.  Obstacle-aware BFS (Dijkstra
//      h=0, same primitive as RoutePathSearch::snapToRoute) finds
//      the nearest REACHABLE candidate — unreachable ones (still
//      inside the obstacle) are skipped automatically.
//   4. Output = detour grid path (robot → candidate) + reference
//      suffix from that candidate onward.
//   5. No candidate reachable → warn + pass-through (upper layers
//      handle the stuck condition).
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
    /// Scan the reference path forward from the waypoint nearest to the
    /// robot (up to scan_distance_m) and return the index of the first
    /// blocked waypoint (!isWalkable OR cost >= blockage_threshold).
    std::optional<size_t> findBlockage(
        const Context& ctx, const IMapView& map_view) const;

    // ── parameters ────────────────────────────────────────────
    double scan_distance_m_    = 5.0;   // forward scan for blockage detection
    double blockage_threshold_ = 0.8;   // soft blockage: cost >= this triggers a detour
    double reconnect_offset_m_ = 1.0;   // rejoin the path at least this far past the blockage
    double obstacle_cost_weight_ = 1.0;
    double max_planning_time_  = 0.1;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::DetourPlanner, detour_planner)
