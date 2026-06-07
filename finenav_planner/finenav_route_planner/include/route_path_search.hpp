// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_route_planner/route_params.hpp"

#include "astar_algorithm.hpp"   // findPathToAny (h=0 Dijkstra) for nearest-node search
#include "route/path_converter.hpp"
#include "route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"
#include "route/route_planner.hpp"

namespace finenav {

// TODO： 现在astar和bfs完全是两份copy的代码，状态糟糕，实际上它们唯一的差别就是中间的数据结构
// =============================================================
// RoutePathSearch
//
// FineNav planner that navigates on a topological route graph.
//
// Map interface:
//   Implementors provide IGraphView, which supplies the GeoJSON
//   graph string. No occupancy grid is needed.
//
// Planning strategy (plan()):
//   1. Load graph from graph_view.getGraphJson().
//   2. BFS-search for the nearest route node to ctx.robot_state.pose.
//   3. BFS-search for the nearest route node to ctx.goal_pose.
//   4. Dijkstra (RoutePlanner) from start node to goal node.
//   5. Densify with PathConverter and return trajectory.
// =============================================================

class RoutePathSearch {
public:
    RoutePathSearch() = default;

    using ConfigType = route_planner::Params;

    static constexpr IOProfile InputProfile = {
        .pose  = true,    // needs current robot pose for nearest-node BFS
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

    // ── Graph interface ───────────────────────────────────────
    class IGraphView {
    public:
        virtual ~IGraphView() = default;
        /// Returns the GeoJSON string of the route graph to plan on.
        virtual const std::string& getGraphJson() const = 0;
    };

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputDataT<RoutePathSearch>>;

    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IGraphView& graph_view);

private:
    // ── Nearest-node BFS helper ───────────────────────────────
    struct NodeCandidate {
        unsigned int node_id     = 0;
        unsigned int graph_index = 0;
        double       distance    = 0.0;
    };

    NodeCandidate findNearestNode(
        const finenav_route::Graph&        graph,
        const geometry_msgs::msg::Pose&    pose) const;

    // ── Components ────────────────────────────────────────────
    finenav_route::GeoJsonGraphFileLoader loader_;
    finenav_route::RoutePlanner           route_planner_;
    finenav_route::PathConverter          path_converter_;

    // ── Parameters ────────────────────────────────────────────
    double connector_density_ = 0.05;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::RoutePathSearch, route_planner)



