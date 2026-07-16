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
//   2. Dijkstra on the route graph (route::findRoute) → route poses
//      plus the node sequence along the route.
//   3. Start prune: if the robot already projects positively onto the
//      first route edge (node_seq[0] → node_seq[1]), it is past the
//      first node — entering there would mean backtracking.  Snap the
//      robot onto that edge instead (virtual node + edge split on a
//      graph copy) and re-run findRoute from the virtual node.
//   4. Output = start connector + route + reversed goal connector.
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

    // Result of snapping a world pose onto one specific directed edge.
    struct EdgeSnap {
        size_t     edge_from{SIZE_MAX};   // source node of the edge
        size_t     edge_idx{SIZE_MAX};    // index in graph.adj[edge_from]
        size_t     pt_idx{SIZE_MAX};      // polyline index of the snap point;
                                          // SIZE_MAX = projection fallback
                                          // (snap_point is not a polyline vertex)
        Position3D snap_point{};          // world coordinates of the entry point
        std::vector<Position3D> connector_path;   // world-frame, pose → snap_point
        bool valid() const { return edge_from != SIZE_MAX; }
    };

private:
    /// @brief Obstacle-aware search (Dijkstra h=0 on the occupancy map) for
    ///        the nearest reachable graph node.  Returns an invalid NodeSnap
    ///        when no node is reachable.
    NodeSnap snapToNearestNode(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    /// @brief Same search restricted to the intermediate polyline points of
    ///        edge(edge_from → edge_to).  Falls back to a pure-geometry
    ///        projection onto the edge chord when the edge has no
    ///        intermediate points.  Used by the start prune in plan().
    EdgeSnap snapToEdge(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view,
        size_t                          edge_from,
        size_t                          edge_to) const;

    /// @brief Insert a virtual node at snap.snap_point and split the snapped
    ///        edge (and its reverse edge, if present) around it.  Costs are
    ///        pro-rated by sub-polyline length.  Returns the virtual node index.
    size_t insertSnapNode(route::Graph& graph, const EdgeSnap& snap) const;

    /// @brief Split the directed edge graph.adj[from][edge_idx] at world point
    ///        `p`: from → virtual_idx replaces the original edge and
    ///        virtual_idx → old-target is appended.  If `p` is not an existing
    ///        polyline vertex it is first inserted into the closest segment.
    void splitDirectedEdge(
        route::Graph&     graph,
        size_t            from,
        size_t            edge_idx,
        const Position3D& p,
        size_t            virtual_idx) const;

    // ── Cached graph & Mutex ──────────────────────────────────
    mutable std::unique_ptr<std::mutex> graph_mutex_ = std::make_unique<std::mutex>();
    std::optional<route::Graph> cached_graph_;

    // ── Parameters ────────────────────────────────────────────
    double path_density_ = 0.05;
    double obstacle_cost_weight_ = 1.0;
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
