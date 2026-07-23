// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/astar/astar_algorithm.hpp"

#include <algorithm>
#include <chrono>

namespace astar {

using finenav::Vec3;
using finenav::Vec3Hash;
using finenav::Heuristic;
using finenav::Connectivity;
using finenav::calculateHeuristic;
using finenav::getNeighbors;
using finenav::calculateStepCost;
using finenav::cellDistance;

// ===== 构造 =====

AStarAlgorithm::AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost)
    : is_walkable_(std::move(is_walkable)),
      get_cost_(std::move(get_cost))
{}

// ===== 标准 A* 搜索（固定目标点）=====

std::optional<std::vector<Vec3>> AStarAlgorithm::findPath(
    const Vec3& start,
    const Vec3& goal)
{
    // 起点或目标不可通行，直接返回失败
    if (!is_walkable_(start) || !is_walkable_(goal))
        return std::nullopt;

    // 重置搜索状态
    open_set_ = {};
    visited_.clear();
    came_from_.clear();

    int  iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    // 记录目标容忍范围内的最佳候选节点（若无法精确到达目标时使用）
    std::optional<AstarNode> best_goal_node;
    double best_goal_dist = std::numeric_limits<double>::infinity();

    // 初始化起点节点：g=0, h=启发式估计
    AstarNode start_node(start);
    start_node.g      = 0.0;
    start_node.h      = calculateHeuristic(heuristic_, start, goal);
    start_node.updateF();
    start_node.parent = start;

    open_set_.push(start_node);
    came_from_[start] = start_node;

    // 主搜索循环
    while (!open_set_.empty())
    {
        // 迭代次数限制检查
        if (max_iterations_ >= 0 && iterations++ > max_iterations_)
            break;

        // 时间限制检查
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_)
            break;

        // 弹出 f 值最小的节点
        AstarNode current = open_set_.top();
        open_set_.pop();

        // 跳过已扩展节点（关闭集去重）
        if (visited_.contains(current.pos)) continue;
        visited_.insert(current.pos);

        // 精确到达目标 → 回溯路径
        if (current.pos == goal)
            return reconstructPath(start, goal);

        // 记录容忍范围内的最佳候选
        double goal_dist = cellDistance(current.pos, goal);
        if (goal_dist < best_goal_dist) {
            best_goal_dist = goal_dist;
            best_goal_node = current;
        }

        // 扩展所有可通行的未访问邻居
        for (const Vec3& nb_pos : getNeighbors(current.pos, connectivity_))
        {
            if (!is_walkable_(nb_pos))     continue;
            if (visited_.contains(nb_pos)) continue;

            // 边代价 = 步长代价 × (movement_weight_ + terrain_weight_ × 地形代价)
            double step_cost   = calculateStepCost(current.pos, nb_pos);
            double map_cost    = get_cost_(nb_pos);
            double tentative_g = current.g + step_cost * (movement_weight_ + terrain_weight_ * map_cost);

            // 发现更短路径或首次到达 → 更新节点并入队
            if (!came_from_.contains(nb_pos) ||
                tentative_g < came_from_[nb_pos].g)
            {
                AstarNode nb(nb_pos);
                nb.g      = tentative_g;
                nb.h      = calculateHeuristic(heuristic_, nb_pos, goal) * greedy_weight_;
                nb.updateF();
                nb.parent = current.pos;

                came_from_[nb_pos] = nb;
                open_set_.push(nb);
            }
        }
    }

    // 未精确到达目标，但在容忍范围内有候选 → 使用最佳候选
    if (best_goal_node.has_value() && best_goal_dist <= goal_tolerance_)
        return reconstructPath(start, best_goal_node->pos);

    return std::nullopt;
}

// ===== 路径回溯 =====
// 从 goal 沿 came_from_ parent 链回溯到 start，reverse 后返回正向序列。
// 调用方保证 goal 在 came_from_ 中（搜索已成功到达），否则 at() 抛异常。

std::vector<Vec3> AStarAlgorithm::reconstructPath(const Vec3& start, const Vec3& goal) const
{
    std::vector<Vec3> path;
    Vec3 cur = goal;
    while (true) {
        path.push_back(cur);
        if (cur == start) break;
        cur = came_from_.at(cur).parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ===== Dijkstra 模式搜索（h=0, 动态目标集）=====

/**
 * @details 此函数将启发式强制设为 0，退化为 Dijkstra 均匀代价扩展。
 *          原因：在没有固定目标的情况下无法计算有效的可采纳启发式，
 *          使用 h=0 保证最优性（等价于 Dijkstra 算法）。
 *
 *          当 accept_goal 不为空时：
 *          搜索到的目标节点若被 accept_goal 拒绝，则记录为备用（fallback），
 *          继续搜索直到找到被接受的目标或队列耗尽。
 *          若最终无目标被接受，返回 g 值最小的备用目标。
 */
std::optional<std::vector<Vec3>> AStarAlgorithm::findPathToAny(
    const Vec3& start,
    IsGoalFn    is_goal,
    GoalFilterFn accept_goal)
{
    // 起点不可通行，直接返回失败
    if (!is_walkable_(start))
        return std::nullopt;

    // 重置搜索状态
    open_set_ = {};
    visited_.clear();
    came_from_.clear();

    int  iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    // 初始化起点节点：g=0, h=0（Dijkstra 模式，无启发式）
    AstarNode start_node(start);
    start_node.g      = 0.0;
    start_node.h      = 0.0;   // 强制为 0
    start_node.updateF();
    start_node.parent = start;

    open_set_.push(start_node);
    came_from_[start] = start_node;

    std::optional<Vec3> best_fallback;   // 被拒绝的目标中 g 值最小的备用目标

    // 主搜索循环（与标准 A* 结构相同，但 h=0）
    while (!open_set_.empty())
    {
        // 迭代次数限制检查
        if (max_iterations_ >= 0 && iterations++ > max_iterations_)
            break;

        // 时间限制检查
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_)
            break;

        AstarNode current = open_set_.top();
        open_set_.pop();

        if (visited_.contains(current.pos)) continue;
        visited_.insert(current.pos);

        // 找到满足 is_goal 的节点
        if (is_goal(current.pos))
        {
            // 通过过滤器 → 接受该目标，停止搜索
            if (!accept_goal || accept_goal(current.pos))
                return reconstructPath(start, current.pos);

            // 被拒绝 → 记录为备用（仅保留第一个=最小 g 的，因搜索按 g 递增扩展）
            if (!best_fallback.has_value())
                best_fallback = current.pos;
        }

        // 扩展邻居（与标准 A* 相同逻辑）
        for (const Vec3& nb_pos : getNeighbors(current.pos, connectivity_))
        {
            if (!is_walkable_(nb_pos))     continue;
            if (visited_.contains(nb_pos)) continue;

            double step_cost   = calculateStepCost(current.pos, nb_pos);
            double map_cost    = get_cost_(nb_pos);
            double tentative_g = current.g + step_cost * (movement_weight_ + terrain_weight_ * map_cost);

            if (!came_from_.contains(nb_pos) ||
                tentative_g < came_from_[nb_pos].g)
            {
                AstarNode nb(nb_pos);
                nb.g      = tentative_g;
                nb.h      = 0.0;   // h=0, Dijkstra 模式
                nb.updateF();
                nb.parent = current.pos;

                came_from_[nb_pos] = nb;
                open_set_.push(nb);
            }
        }
    }

    // 无目标被接受 → 返回 g 值最小的备用目标
    if (best_fallback.has_value())
        return reconstructPath(start, *best_fallback);

    return std::nullopt;
}

} // namespace astar
