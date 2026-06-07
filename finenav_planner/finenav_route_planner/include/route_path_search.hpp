// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_route_planner/route_params.hpp"

#include "astar_algorithm.hpp"
#include "route/route_graph.hpp"
#include "route/route_search.hpp"

namespace finenav {

// =============================================================
// RoutePathSearch
//
// FineNav planner for topological-graph route planning.
//
// Planning flow in plan():
//   1. Parse the GeoJSON graph from IGraphView.
//   2. AStarAlgorithm::findPathToAny (h=0, virtual cell grid) to
//      find the nearest route node to ctx.robot_state.pose.
//   3. Same search for ctx.goal_pose.
//   4. Dijkstra on the route graph (route::findRoute).
//   5. Return the densified pose list.
// =============================================================

class RoutePathSearch {
public:
    RoutePathSearch() = default;

    using ConfigType = route_planner::Params;

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

    // IGraphView: provide the GeoJSON string.
    class IGraphView {
    public:
        virtual ~IGraphView() = default;
        virtual const std::string& getGraphJson() const = 0;
    };

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputDataT<RoutePathSearch>>;

    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IGraphView& graph_view);

private:
    // BFS (Dijkstra h=0) on a virtual cell grid to find the nearest node.
    // Returns the node's index in graph.nodes, or SIZE_MAX on failure.
    size_t findNearestNodeIndex(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose) const;

    double connector_density_ = 0.05;   ///< cell size (m) for nearest-node BFS
    double path_density_      = 0.05;   ///< interpolation step (m) for output path
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::RoutePathSearch, route_planner)
