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
// and the goal pose to the nearest reachable route graph node.
//
// The route graph (GeoJSON) is loaded from the path configured via
// the `graph_file` dynamic parameter in route.yaml; it is cached
// across planning calls and reloaded only when the parameter changes.
//
// Planning flow in plan():
//   1. Use cached (or freshly loaded) route graph.
//   2. AStarAlgorithm::findPathToAny (h=0 + map isWalkable/getCost)
//      → find the nearest REACHABLE route node to ctx.robot_state.pose.
//   3. Same search for ctx.goal_pose.
//   4. Dijkstra on the route graph (route::findRoute).
//   5. Return the densified pose list.
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
    // the nearest route node so that the connector path avoids obstacles.
    using IMapView = astar::IMapView;

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputDataT<RoutePathSearch>>;

    // Loads (or reloads) the graph when graph_file param changes.
    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

    /**
     * @brief 从 GeoJSON 格式的字符串更新缓存的路网图
     * @param geojson_str GeoJSON 格式的数据内容
     */
    void updateGraph(const std::string& geojson_str);

    // Result of snapping a world pose to the nearest reachable route node.
    // Carries both the node index and the obstacle-aware grid path (in world
    // frame) that connects the pose to that node.
    struct SnapResult {
        size_t                    node_index{SIZE_MAX};
        std::vector<Position3D>   connector_path;   // world-frame, pose → node
    };

private:
    // Obstacle-aware BFS (Dijkstra h=0 on the occupancy map) to snap a
    // world pose to the nearest reachable route graph node.
    // Returns a SnapResult with both the node index and the world-frame
    // connector path from `pose` to that node.
    SnapResult findNearestNodeIndex(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    // ── Cached graph & Mutex ──────────────────────────────────
    mutable std::unique_ptr<std::mutex> graph_mutex_ = std::make_unique<std::mutex>();
    std::optional<route::Graph> cached_graph_;

    // ── Parameters ────────────────────────────────────────────
    double path_density_ = 0.05;
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
