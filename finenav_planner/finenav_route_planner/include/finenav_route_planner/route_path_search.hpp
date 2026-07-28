// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>
#include <mutex>
#include <std_msgs/msg/string.hpp>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/route_params.hpp"

#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/common/imap_view.hpp"
#include "finenav_route_planner/common/snap_search_utils.hpp"
#include "finenav_route_planner/route/route_graph.hpp"
#include "finenav_route_planner/route/route_search.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

/**
 * @brief 拓扑路网图（route graph）路径规划器（FineNav 框架适配层）
 * @details 从使用者角度看，本规划器与 AstarPathSearch / BfsPathSearch 接收相同的
 *          IMapView。占据地图用于在把机器人当前位姿与目标位姿吸附（snap）到
 *          最近可达路网图点（节点，或某条边 polyline 上的中间点）时，
 *          引导避障 BFS 搜索。
 * @details 路网图（GeoJSON）从 /geojson 话题加载；它在多次规划调用之间被缓存，
 *          当话题发布新数据时重新加载。
 * @details plan() 中的规划流程：
 *          1. 使用缓存的路网图。
 *          2. AStarAlgorithm::findPathToAny（h=0 + 地图 isWalkable/getCost）
 *             → 找到距 ctx.robot_state.pose 最近的可达图点（节点或路径点）。
 *          3. 对 ctx.goal_pose 执行同样的搜索。
 *          4. 对路径点吸附：在可变图副本上插入虚拟节点并分裂边。
 *          5. 在（可能已修改的）路网图上运行 Dijkstra（route::findRoute）。
 *          6. 返回稠密化后的位姿序列。
 */

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

    // 与 AstarPathSearch 和 BfsPathSearch 使用相同的地图接口。
    // isWalkable / getCost 在将位姿吸附到最近路网节点的 BFS 中使用，
    // 以使连接路径能够避开障碍物。
    using IMapView = finenav::IMapView;

    using Context = PlanningContext<RoutePathSearch>;
    using Result  = std::optional<OutputTrajectoryT<RoutePathSearch>>;

    /**
     * @brief 根据动态参数配置规划器
     * @param params 参数结构体（由 generate_parameter_library 生成）
     * @details 读取 path_density / min_approach_angle_deg / terrain_weight 三个参数。
     *          路网图不在此处加载——实际通过 /geojson 话题到达时由 updateGraph() 异步更新。
     */
    void configure(const ConfigType& params);
    void reset() {}

    Result plan(const Context& ctx, const IMapView& map_view);

    /**
     * @brief 从 GeoJSON 格式的字符串更新缓存的路网图
     * @param geojson_str GeoJSON 格式的数据内容
     */
    void updateGraph(const std::string& geojson_str);

    /**
     * @brief 将世界坐标位姿吸附到最近可达路网图点的结果
     * @details 图点可以是节点，也可以是某条边 polyline 上的中间点。
     *          结构体携带入口点信息，以及连接该位姿与入口点的
     *          避障栅格路径（世界坐标系）。
     */
    struct SnapResult {
        // 节点吸附 — 当 isNode() 为 true 时有效。
        size_t node_index{SIZE_MAX};

        // 路径点吸附 — 当 isPath() 为 true 时有效。
        // 吸附点落在边 graph.adj[edge_from][edge_idx] 的某个 polyline
        // 中间点上。snap_point 保存该点的世界坐标。
        size_t      edge_from{SIZE_MAX};
        size_t      edge_to{SIZE_MAX};       // 边的目标节点
        size_t      edge_idx{SIZE_MAX};      // 在 graph.adj[edge_from] 中的下标
        size_t      pt_idx{SIZE_MAX};        // 在该边 polyline 中的下标
        Position3D  snap_point{};           // 入口点的世界坐标

        std::vector<Position3D> connector_path;   // 世界坐标系，pose → snap_point

        bool isNode() const { return node_index != SIZE_MAX; }
        bool isPath() const { return edge_from != SIZE_MAX; }
    };

    /// @brief Snap-to-route-curve result — used for route end-trimming.
    ///        Finds the nearest reachable point on the densified route curve
    ///        (obstacle-aware BFS on the occupancy map).  When valid, the
    ///        robot or goal connector can be projected to route[index] instead
    ///        of the node centre, eliminating the backtracking that node-level
    ///        entry/exit would cause.
    struct RouteSnap {
        size_t index{SIZE_MAX};                      ///< index into the route poses
        std::vector<Position3D> connector_path;     ///< world-frame, pose → route[index]
        bool valid() const { return index != SIZE_MAX; }
    };

private:
    /**
     * @brief 避障 BFS（在占据地图上运行 h=0 的 Dijkstra），
     *        将世界坐标位姿吸附到最近可达的路网图点（节点或路径点）
     * @param graph 路网图
     * @param pose 待吸附的世界坐标位姿
     * @param map_view 地图视图（提供可通行性与代价查询）
     * @return SnapResult，同时包含入口点信息以及从 `pose` 到该点的
     *         世界坐标系连接路径
     */
    SnapResult findNearestGraphPoint(
        const route::Graph&             graph,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    /**
     * @brief 在可变图的给定世界坐标位置插入一个临时虚拟节点
     * @param graph 可变的路网图
     * @param pos 虚拟节点的世界坐标位置
     * @return 新节点的下标
     * @note 不会把该节点注册到 id_index（虚拟节点没有 id）
     */
    size_t insertVirtualNode(
        route::Graph&         graph,
        const Position3D&     pos) const;

    /**
     * @brief 分裂路径点吸附的 SnapResult 所落在的那条边
     * @param graph 可变的路网图
     * @param snap 路径点吸附结果
     * @param virtual_idx 虚拟节点的下标
     * @details 将原有向边（graph.adj[edge_from][edge_idx]）替换为两条子边：
     *          edge_from → virtual_idx 和 virtual_idx → edge_to。
     *          若存在反向边（edge_to → edge_from），也一并分裂。
     */
    void splitEdgeForSnap(
        route::Graph&         graph,
        const SnapResult&     snap,
        size_t                virtual_idx) const;

    /// @brief Obstacle-aware search (Dijkstra h=0 on the occupancy map) for
    ///        the nearest reachable point on the densified route curve.  Used
    ///        to trim the route curve at both ends: the robot enters at its
    ///        nearest reachable route point and the path leaves the route at
    ///        the point nearest to the goal.
    RouteSnap snapToRoute(
        const route::RoutePoses&        route,
        size_t                          begin_idx,
        const geometry_msgs::msg::Pose& pose,
        const IMapView&                 map_view) const;

    // ── 缓存路网图与互斥锁 ──────────────────────────────────
    mutable std::unique_ptr<std::mutex> graph_mutex_ = std::make_unique<std::mutex>();
    std::optional<route::Graph> cached_graph_;

    // ── 配置参数 ────────────────────────────────────────────
    double path_density_ = 0.05;
    double min_approach_angle_deg_ = 0.0;
    double terrain_weight_ = 1.0;          // 地形代价乘数（吸附搜索用）
    /// When false, output only the on-network route segment (entry/exit
    /// trimming still applies).  For pipelines where a downstream
    /// DetourPlanner re-anchors the robot to the reference every cycle —
    /// a connector baked into a single-shot plan goes stale.
    bool   include_connectors_ = true;

    // ── 吸附搜索参数 ────────────────────────────────────────
    finenav::Connectivity snap_connectivity_ = finenav::Connectivity::N8;
    int    snap_max_iterations_    = -1;    // -1 表示不限制
    double snap_max_planning_time_ = 5.0;   // 秒
};

} // namespace finenav

// RoutePathSearch 手动特化 AlgoConfigurator 而非使用 FINENAV_REGISTER_ALGO_CONFIG 宏。
// 原因：宏只生成 ParamListener + setUserCallback，而 RoutePathSearch 除此之外还需要
// 订阅 /geojson 话题以异步加载路网图。两者生命周期统一由 LifetimeKeeper 管理。
namespace finenav::core {

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

} // namespace finenav::core
