// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>
#include <mutex>
#include <std_msgs/msg/string.hpp>

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_route_planner/route_params.hpp"

#include "astar_algorithm.hpp"
#include "imap_view.hpp"
#include "route/route_graph.hpp"
#include "route/route_search.hpp"

namespace finenav {

// =============================================================
// RoutePathSearch
//
// FineNav planner for topological-graph route planning.
//
// From the user's perspective this planner takes the same IMapView
// as AstarPathSearch / BfsPathSearch.  The occupancy map is used to
// guide obstacle-aware BFS when snapping the robot's current pose
// and the goal pose to the route graph.
//
// The route graph (GeoJSON) is loaded from the /geojson topic;
// it is cached across planning calls and reloaded when the topic
// publishes new data.
//
// Planning flow in plan():
//   1. Snap robot pose and goal pose to the nearest REACHABLE graph
//      node (obstacle-aware Dijkstra on the occupancy map, h = 0).
//   2. Dijkstra on the route graph (route::findRoute) → densified
//      route curve.
//   3. Trim both ends: project the robot and the goal onto the route
//      curve (obstacle-aware BFS to the nearest reachable route point)
//      and keep only the segment in between.  This removes the
//      backtracking that entering/leaving at node centres would cause,
//      no matter how many nodes the robot has already passed.
//   4. Output = start connector + route segment + reversed goal
//      connector.  With include_connectors=false only the route
//      segment is emitted — for pipelines where a downstream
//      DetourPlanner re-anchors the robot to the reference every
//      cycle (a connector baked into a single-shot plan goes stale).
// =============================================================

class RoutePathSearch {
public:
    RoutePathSearch() = default;

    using ConfigType = route_planner::Params;

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

    // Same map interface as AstarPathSearch and BfsPathSearch.
    // isWalkable / getCost are used during the BFS that snaps poses to
    // the route graph so that the connector path avoids obstacles.
    using IMapView = astar::IMapView;

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputDataT<RoutePathSearch>>;

    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

    /**
     * @brief 从 GeoJSON 格式的字符串更新缓存的路网图
     * @param geojson_str GeoJSON 格式的数据内容
     */
    void updateGraph(const std::string& geojson_str);

    // Result of snapping a world pose to the nearest reachable graph node.
    struct NodeSnap {
        size_t node_index{SIZE_MAX};
        std::vector<Position3D> connector_path;   // world-frame, pose → node
        bool valid() const { return node_index != SIZE_MAX; }
    };

    // Result of snapping a world pose to the nearest reachable point of the
    // densified route curve.
    struct RouteSnap {
        size_t index{SIZE_MAX};                    // index into the route poses
        std::vector<Position3D> connector_path;   // world-frame, pose → route[index]
        bool valid() const { return index != SIZE_MAX; }
    };

private:
    /// @brief Obstacle-aware search (Dijkstra h=0 on the occupancy map) for
    ///        the nearest reachable graph node.  Returns an invalid NodeSnap
    ///        when no node is reachable.
    NodeSnap snapToNearestNode(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    /// @brief Same search over the points of route[begin_idx..end).  Used by
    ///        plan() to trim the route curve at both ends: the robot enters
    ///        at its nearest reachable route point and the path leaves the
    ///        route at the point nearest to the goal.
    RouteSnap snapToRoute(
        const route::RoutePoses&        route,
        size_t                          begin_idx,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    // ── Cached graph & Mutex ──────────────────────────────────
    mutable std::unique_ptr<std::mutex> graph_mutex_ = std::make_unique<std::mutex>();
    std::optional<route::Graph> cached_graph_;

    // ── Parameters ────────────────────────────────────────────
    double path_density_ = 0.05;
    double obstacle_cost_weight_ = 1.0;
    // When false, output only the on-network segment (entry/exit trimming
    // still applies) — for pipelines where DetourPlanner attaches the robot.
    bool   include_connectors_ = true;
};

} // namespace finenav

//TODO: 不用 FINENAV_REGISTER_ALGO_CONFIG(finenav::RoutePathSearch, route_planner) 的做法有待商榷
namespace finenav {

template <>
struct AlgoConfigurator<finenav::RoutePathSearch> {
    static constexpr std::string_view name = "route_planner";
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, finenav::RoutePathSearch& algo) {
        auto listener = std::make_shared<route_planner::ParamListener>(node, "route_planner");
        algo.configure(listener->get_params());
        listener->setUserCallback([&algo](const auto& params) {
            algo.configure(params);
        });

        struct LifetimeKeeper {
            std::shared_ptr<route_planner::ParamListener> listener;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
        };

        auto keeper = std::make_shared<LifetimeKeeper>();
        keeper->listener = listener;
        keeper->sub = node->create_subscription<std_msgs::msg::String>(
            "/geojson",
            rclcpp::QoS(1).transient_local(),
            [&algo](const std_msgs::msg::String::SharedPtr msg) {
                algo.updateGraph(msg->data);
            }
        );

        return keeper;
    }
};

} // namespace finenav
