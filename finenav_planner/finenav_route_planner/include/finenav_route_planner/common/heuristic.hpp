// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <algorithm>
#include "finenav_route_planner/common/vec3.hpp"

namespace finenav {

/// 启发函数类型（作用于 Vec3 栅格坐标）
enum class Heuristic {
    // h = 0：使 A* 退化为 Dijkstra 算法（加权 BFS）。
    // 在均匀代价栅格上等价于经典 BFS。
    // 在加权栅格上可在无方向引导的情况下找到最小代价路径，
    // 代价是需要扩展更多节点。
    Zero,
    Euclidean,   // 欧氏距离
    Manhattan,   // 曼哈顿距离
    Chebyshev,   // 切比雪夫距离
    // 三维 octile（对角线）距离。当对角步代价 = √2（三维角步代价 = √3）时，
    // 对 8 连通栅格具有可采纳性。
    // 推荐作为 D* Lite 的默认启发式。
    Diagonal
};

/**
 * @brief 按给定启发式类型计算 from 到 to 的启发式代价
 * @param type 启发函数类型
 * @param from 起始栅格坐标
 * @param to 目标栅格坐标
 * @return 启发式估计代价（栅格单位）
 * @note 启发式作用于 Vec3 栅格坐标；对 2D 栅格 z 恒为 0，三维公式可正确退化为二维
 */
inline double calculateHeuristic(
    Heuristic   type,
    const Vec3& from,
    const Vec3& to)
{
    switch (type)
    {
    case Heuristic::Zero:
        return 0.0;
    case Heuristic::Euclidean: {
        double dx = from.x - to.x, dy = from.y - to.y, dz = from.z - to.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    case Heuristic::Manhattan:
        return std::abs(from.x - to.x) + std::abs(from.y - to.y) + std::abs(from.z - to.z);
    case Heuristic::Chebyshev:
        return std::max({std::abs(from.x - to.x),
                         std::abs(from.y - to.y),
                         std::abs(from.z - to.z)});
    case Heuristic::Diagonal: {
        // Octile / 对角线距离：2 轴同时步进的代价为 √2。
        // 与 D* Lite 共享（8 连通栅格的推荐默认启发式）。
        const double dx = std::abs(from.x - to.x);
        const double dy = std::abs(from.y - to.y);
        const double dz = std::abs(from.z - to.z);
        const double dmin = std::min({dx, dy, dz});
        const double dmax = std::max({dx, dy, dz});
        const double dmid = dx + dy + dz - dmin - dmax;
        return dmin * std::sqrt(3.0)
             + (dmid - dmin) * std::sqrt(2.0)
             + (dmax - dmid);
    }
    default:
        return 0.0;
    }
}

} // namespace finenav

