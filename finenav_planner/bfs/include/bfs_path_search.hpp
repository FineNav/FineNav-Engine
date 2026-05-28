// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/concepts.hpp"
#include "finenav_planner_bfs/bfs_params.hpp"
#include "finenav_util/algo_configurator.hpp"

#include "bfs_algorithm.hpp"
#include "bfs_imap_view.hpp"

namespace finenav {

// =============================================================
// BfsPathSearch
//
// FineNav planner that performs uniform-grid BFS path search.
// It is useful when every walkable grid cell has the same cost and
// the desired path is the fewest number of grid hops.
// =============================================================

class BfsPathSearch {
public:
    BfsPathSearch() = default;

    using ConfigType = bfs_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose = true,
        .vel = false,
        .accel = false,
        .time = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose = true,
        .vel = false,
        .accel = false,
        .time = false
    };

    using IMapView = bfs::IMapView;

    using Context = PlanningContext<BfsPathSearch>;
    using Result = std::optional<OutputDataT<BfsPathSearch>>;

    void configure(const ConfigType& params);

    Result plan(const Context& ctx, const IMapView& map_view);

private:
    bfs::BfsAlgorithm::Connectivity connectivity_ = bfs::BfsAlgorithm::Connectivity::N8;
    double goal_tolerance_m_ = 0.5;
    int max_iterations_ = 1000000;
    double max_planning_time_ = 5.0;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::BfsPathSearch, bfs_planner)
