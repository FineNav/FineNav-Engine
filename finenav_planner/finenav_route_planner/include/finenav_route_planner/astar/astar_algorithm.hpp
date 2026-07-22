// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <optional>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>

#include "finenav_route_planner/common/grid_search_utils.hpp"   // Connectivity, getNeighbors, calculateStepCost,
                                    // cellDistance, reconstructPathImpl
#include "finenav_route_planner/common/heuristic.hpp"

namespace astar {

// ── 从 finenav 引入通用栅格搜索类型 ──────────────────────────────────
using finenav::Vec3;
using finenav::Vec3Hash;
using finenav::Heuristic;
using finenav::Connectivity;
using finenav::calculateHeuristic;
using finenav::getNeighbors;
using finenav::calculateStepCost;
using finenav::cellDistance;
using finenav::reconstructPathImpl;

/**
 * @brief A* 搜索的内部节点结构，统一使用 Vec3 坐标
 * @details 每个节点记录三个核心代价：
 *          - g: 从起点到当前节点的实际路径代价
 *          - h: 从当前节点到目标的启发式估计代价
 *          - f = g + h: 节点优先级，f 越小越优先扩展
 * @note parent == pos 表示该节点为起点（自哨兵标记）
 */
struct AstarNode {
    Vec3   pos;                                // 栅格坐标
    double g = std::numeric_limits<double>::max();  // 实际路径代价
    double h = 0.0;                                 // 启发式估计代价
    double f = std::numeric_limits<double>::max();  // 总代价 f = g + h
    Vec3   parent{};                                // 父节点坐标（parent == pos 表示起点）

    AstarNode() = default;
    explicit AstarNode(const Vec3& p) : pos(p) {}
    /// 刷新 f 值：f = g + h
    void updateF() { f = g + h; }
};

/// 优先队列比较器：f 值小的节点优先弹出（最小堆）
struct AstarNodeCompare {
    bool operator()(const AstarNode& a, const AstarNode& b) const
    { return a.f > b.f; }
};

/**
 * @brief 标准均匀栅格 A* 搜索引擎
 * @details A* 是一种启发式图搜索算法，结合了 Dijkstra 的最优性保证和贪心最佳优先搜索的效率。
 *          核心思想是使用 f = g + h 作为节点优先级：
 *          - g(s): 从起点到节点 s 的实际代价
 *          - h(s): 从节点 s 到目标的启发式估计（必须满足可采纳性以保证最优）
 *          - f(s): 总估计代价，用于优先队列排序
 * @details 与 D* Lite 的本质区别：A* 是无状态的，每次 findPath() 调用从零开始搜索，
 *          不保留跨调用的状态。因此当地图频繁变化时，A* 需要每次重新搜索全图。
 * @note 邻域枚举、步长代价等底层操作委托给 grid_search_utils.hpp 中的共享函数
 * @note 该类独立于 FineNav 框架，可在任何栅格地图场景下直接使用
 */
class AStarAlgorithm {
public:

    using Connectivity = finenav::Connectivity;   // 重导出，方便外部使用

    /// 可通行性回调：传入栅格坐标，返回该格是否可通行
    using IsWalkableFn = std::function<bool(const Vec3&)>;
    /// 地形代价回调：传入栅格坐标，返回该格的额外通行代价（≥0）
    using GetCostFn    = std::function<double(const Vec3&)>;
    /// 目标判定回调：传入栅格坐标，返回是否为搜索目标
    using IsGoalFn     = std::function<bool(const Vec3&)>;

    /**
     * @brief 目标过滤器回调（用于 findPathToAny）
     * @details 当搜索找到一个满足 is_goal 的节点时，调用此回调进行二次判定：
     *          返回 true 表示接受该目标（停止搜索）；
     *          返回 false 表示拒绝该目标（继续搜索，该目标记录为备用）。
     *          传入 nullptr 表示接受所有目标（默认行为）。
     */
    using GoalFilterFn = std::function<bool(const Vec3&)>;

    /**
     * @brief 构造 A* 搜索引擎
     * @param is_walkable 可通行性查询回调
     * @param get_cost 地形代价查询回调
     */
    AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost);

    // ===== 参数设置 =====

    /// 设置邻域连接方式（N4, N8, N6, N18, N26）
    void setConnectivity(Connectivity c)  { connectivity_      = c; }
    /// 设置启发函数类型（Manhattan, Euclidean, Diagonal, Chebyshev）
    void setHeuristic(Heuristic h)        { heuristic_         = h; }
    /// 设置目标容忍距离（栅格单位），0 表示必须精确到达
    void setGoalTolerance(double t)       { goal_tolerance_    = t; }
    /// 设置最大迭代次数，-1 表示无限制
    void setMaxIterations(int m)          { max_iterations_    = m; }
    /// 设置最大规划时间（秒），≤0 表示无限制
    void setMaxPlanningTime(double t)     { max_planning_time_ = t; }
    /// 设置移动代价权重（控制路径长度与地形代价的相对重要性）
    void setMovementWeight(double w)      { movement_weight_   = w; }
    /// 设置启发式贪婪权重（1.0 = 最优，>1.0 = 更快但可能次优）
    void setGreedyWeight(double w)        { greedy_weight_     = w; }
    /// 设置地形代价乘数（放大或缩小地图地形代价的影响）
    void setTerrainWeight(double w)       { terrain_weight_    = w; }

    // ===== 主要搜索接口 =====

    /**
     * @brief 标准 A* 搜索：从起点到固定目标点的路径搜索
     * @param start 起点栅格坐标
     * @param goal 目标栅格坐标
     * @return 成功时返回从起点到目标的栅格坐标序列，失败时返回 std::nullopt
     * @details 若起点或目标不可通行，直接返回失败。
     *          搜索过程中若未精确到达目标但在 goal_tolerance_ 范围内找到最近节点，
     *          则返回该近似目标路径。
     */
    std::optional<std::vector<Vec3>> findPath(const Vec3& start, const Vec3& goal);

    /**
     * @brief Dijkstra 模式搜索（h=0）：搜索到任意满足条件的最近目标
     * @param start 起点栅格坐标
     * @param is_goal 目标判定回调
     * @param accept_goal 目标过滤器回调，nullptr 表示接受所有满足 is_goal 的节点
     * @return 成功时返回路径序列，失败时返回 std::nullopt
     * @details 适用于目标集动态变化的最近邻查询场景（如 Route 规划器搜索最近路网点）。
     *          强制将 h 设为 0，退化为 Dijkstra 均匀代价扩展，保证在有向加权栅格上的最优性。
     *          配置的 heuristic_ 在此模式下被忽略。
     */
    std::optional<std::vector<Vec3>> findPathToAny(
        const Vec3& start,
        IsGoalFn    is_goal,
        GoalFilterFn accept_goal = nullptr);

private:
    /**
     * @brief 路径回溯：从目标节点沿 parent 链回溯到起点
     * @param goal 目标栅格坐标
     * @return 从起点到目标的栅格坐标序列
     * @details 委托给 grid_search_utils.hpp 中的共享模板函数 reconstructPathImpl
     */
    std::vector<Vec3> reconstructPath(const Vec3& goal) const;

    // ===== 注入的回调函数 =====

    IsWalkableFn is_walkable_;  // 可通行性查询回调
    GetCostFn    get_cost_;     // 地形代价查询回调

    // ===== 搜索参数 =====

    Connectivity connectivity_      = Connectivity::N8;       // 邻域连接方式
    Heuristic    heuristic_         = Heuristic::Euclidean;   // 启发函数类型
    double       goal_tolerance_    = 0.25;                   // 目标容忍距离（栅格单位）
    int          max_iterations_    = 1000000;                // 最大迭代次数，-1 无限制
    double       max_planning_time_ = 5.0;                    // 最大规划时间（秒）
    double       movement_weight_   = 1.0;                    // 移动代价权重
    double       greedy_weight_     = 1.001;                  // 启发式贪婪权重（略>1 略微加速）
    double       terrain_weight_    = 1.0;                    // 地形代价乘数

    // ===== 搜索状态（每次 findPath 调用时重置）=====

    std::priority_queue<AstarNode, std::vector<AstarNode>, AstarNodeCompare> open_set_;  // 开放集（最小堆，按 f 排序）
    std::unordered_set<Vec3, Vec3Hash>            visited_;                               // 关闭集（已扩展节点）
    std::unordered_map<Vec3, AstarNode, Vec3Hash> came_from_;                             // 父节点映射（用于路径回溯）
};

} // namespace astar
