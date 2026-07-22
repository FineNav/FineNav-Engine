// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <cmath>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_route_planner/astar_params.hpp"

#include "finenav_route_planner/astar/astar_algorithm.hpp"
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
 * @brief A* 均匀栅格路径搜索规划器（FineNav 框架适配层）
 * @details 该类是 A* 算法在 FineNav 框架中的 PlannerConcept 实现，
 *          每次 plan() 调用创建一个新的 astar::AStarAlgorithm 实例执行搜索。
 * @details A* 的工作特点：
 *          - 无状态：每次规划从零开始，不保留跨调用的搜索状态
 *          - 最优性：使用可采纳启发式保证找到最优路径（greedy_weight_ = 1.0 时）
 *          - 效率：通过启发式引导搜索方向，比 Dijkstra 扩展更少节点
 * @details 适用场景：
 *          - 静态地图中的一次性路径规划
 *          - 地图不频繁变化时的路径搜索
 *          - 若地图频繁变化需要反复重规划，考虑使用 D* Lite
 * @note 复用标准的 finenav::IMapView 接口（与 BFS / Route 规划器共享）
 */
class AstarPathSearch {
public:
    AstarPathSearch() = default;

    // ===== FineNav 框架元数据 =====

    /// 参数类型（由 generate_parameter_library 从 astar.yaml 生成）
    using ConfigType = astar_planner::Params;

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

    /// 复用标准地图视图接口（与 BFS / Route 规划器共享）
    using IMapView = finenav::IMapView;

    using Context = PlanningContext<AstarPathSearch>;
    using Result  = std::optional<OutputTrajectoryT<AstarPathSearch>>;

    // ===== PlannerConcept 接口实现 =====

    /**
     * @brief 根据 YAML 生成的参数配置规划器
     * @param params 参数结构体（由 generate_parameter_library 生成）
     * @details 解析连接方式字符串（"4"/"8"/"6"/"18"/"26"）和启发函数字符串
     *          （"Manhattan"/"Euclidean"/"Chebyshev"/"Diagonal"），
     *          以及目标容忍距离、最大迭代次数、各权重等数值参数
     */
    void configure(const ConfigType& params);

    /// A* 为无状态搜索，reset() 无需执行任何操作
    void reset() {}

    /**
     * @brief 执行 A* 路径规划
     * @param ctx 规划上下文（包含机器人状态、目标位姿等信息）
     * @param map_view 标准地图视图（提供可通行性和代价查询）
     * @return 成功时返回路径位姿序列，失败时返回 std::nullopt
     * @details 每次调用创建一个新的 AStarAlgorithm 实例：
     *          1. 将世界坐标转换为栅格坐标
     *          2. 构建 AStarAlgorithm 引擎并配置参数
     *          3. 执行 findPath() 搜索
     *          4. 将栅格路径转换回世界坐标系位姿序列
     */
    Result plan(const Context& ctx, const IMapView& map_view);

private:
    // ===== 配置参数 =====

    finenav::Connectivity connectivity_    = finenav::Connectivity::N8;       // 邻域连接方式
    finenav::Heuristic    heuristic_       = finenav::Heuristic::Euclidean;   // 启发函数类型
    double goal_tolerance_m_  = 0.5;       // 目标容忍距离（米）
    int    max_iterations_    = 1000000;   // 最大迭代次数
    double max_planning_time_ = 5.0;       // 最大规划时间（秒）
    double movement_weight_   = 1.0;       // 移动代价权重
    double greedy_weight_     = 1.001;     // 启发式贪婪权重（略>1 略微加速）
    double terrain_weight_    = 1.0;       // 地形代价乘数
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::AstarPathSearch, astar_planner)
