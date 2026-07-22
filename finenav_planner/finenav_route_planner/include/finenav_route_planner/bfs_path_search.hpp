// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/bfs_params.hpp"

#include "finenav_route_planner/astar/astar_algorithm.hpp"   // BFS 即启发式为 Zero 的 A*
#include "finenav_route_planner/common/imap_view.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

/**
 * @brief BFS / Dijkstra 均匀栅格路径搜索规划器（FineNav 框架适配层）
 * @details 该类是 FineNav 框架中的 PlannerConcept 实现，
 *          在不使用方向性启发式的情况下搜索最小代价栅格路径（Dijkstra / 加权 BFS）。
 * @details 内部实现为启发式恒为 Heuristic::Zero 的 AStarAlgorithm：
 *          h(n) = 0  →  f(n) = g(n)  →  纯代价驱动扩展。
 * @details 在均匀代价地图上等价于经典 BFS，找到跳数最少的路径；
 *          在加权地图上找到最小通行代价路径（与 Dijkstra 相同）。
 * @note getCost() 会被实际调用，因此地图视图应返回有意义的代价值
 */
class BfsPathSearch {
public:
    BfsPathSearch() = default;

    // ===== FineNav 框架元数据 =====

    /// 参数类型（由 generate_parameter_library 从 bfs.yaml 生成）
    using ConfigType = bfs_planner::Params;

    /// 输入端数据规格：本规划器仅需目标位姿，不需要机器人速度/加速度
    static constexpr IOProfile InputProfile = {
        .pose  = false,
        .vel   = false,
        .accel = false,
        .time  = false
    };
    /// 输出端数据规格：仅输出位姿序列（路径点）
    static constexpr IOProfile OutputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };

    /// 复用标准地图视图接口（与 AStar / Route 规划器共享）
    using IMapView = finenav::IMapView;

    using Context = PlanningContext<BfsPathSearch>;
    using Result  = std::optional<OutputTrajectoryT<BfsPathSearch>>;

    // ===== PlannerConcept 接口实现 =====

    /**
     * @brief 根据 YAML 生成的参数配置规划器
     * @param params 参数结构体（由 generate_parameter_library 生成）
     * @details 解析连接方式字符串（"4"/"8"/"6"/"18"/"26"），
     *          以及目标容忍距离、最大迭代次数、各权重等数值参数
     */
    void configure(const ConfigType& params);
    /// BFS 为无状态搜索，reset() 无需执行任何操作
    void reset() {}

    /**
     * @brief 执行 BFS / Dijkstra 路径规划
     * @param ctx 规划上下文（包含机器人状态、目标位姿等信息）
     * @param map_view 标准地图视图（提供可通行性和代价查询）
     * @return 成功时返回路径位姿序列，失败时返回 std::nullopt
     * @details 每次调用创建一个新的 AStarAlgorithm 实例（启发式恒为 Zero）：
     *          1. 将世界坐标转换为栅格坐标
     *          2. 构建 AStarAlgorithm 引擎并配置参数
     *          3. 执行 findPath() 搜索
     *          4. 将栅格路径转换回世界坐标系位姿序列
     */
    Result plan(const Context& ctx, const IMapView& map_view);

private:
    // ===== 配置参数 =====
    // 所有字段与 AstarPathSearch 一一对应，唯一区别是启发式恒为 Zero
    finenav::Connectivity connectivity_    = finenav::Connectivity::N8;       // 邻域连接方式
    double goal_tolerance_m_  = 0.5;       // 目标容忍距离（米）
    int    max_iterations_    = 1000000;   // 最大迭代次数
    double max_planning_time_ = 5.0;       // 最大规划时间（秒）
    double movement_weight_   = 1.0;       // 移动代价权重
    double terrain_weight_    = 1.0;       // 地形代价乘数
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::BfsPathSearch, bfs_planner)
