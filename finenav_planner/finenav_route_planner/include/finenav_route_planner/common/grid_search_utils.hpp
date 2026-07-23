// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// =============================================================
// grid_search_utils.hpp
//
// AStarAlgorithm 与 BfsAlgorithm 共享的栅格搜索基础组件。
// 集中放置以消除重复代码，并使 A* 与 BFS 的算法差异一目了然：
//
//   • 本文件中的所有内容对 A* 与 BFS 完全相同。
//   • A* 的差异仅在于：priority_queue + AstarNode（g/h/f）。
//   • BFS 的差异仅在于：FIFO 队列 + Vec3 父节点映射。
// =============================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "finenav_route_planner/common/vec3.hpp"

namespace finenav {

// =============================================================
// Connectivity  —  AStarAlgorithm 与 BfsAlgorithm 共享
// =============================================================

/// 邻域连接方式：N4/N8 为二维栅格，N6/N18/N26 为三维栅格
enum class Connectivity { N4, N8, N6, N18, N26 };

// ── 自由函数 ───────────────────────────────────────────────────────────────

/// 返回给定连接方式 c 下 pos 的所有栅格邻居
inline std::vector<Vec3> getNeighbors(const Vec3& pos, Connectivity c)
{
    static const std::array<Vec3, 4> ortho2d = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0}
    }};
    static const std::array<Vec3, 4> diag2d = {{
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}
    }};
    static const std::array<Vec3, 6> face3d = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
    }};
    static const std::array<Vec3, 12> edge3d = {{
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
        {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
        {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1}
    }};
    static const std::array<Vec3, 8> corner3d = {{
        {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
        {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
    }};

    std::vector<Vec3> result;
    switch (c)
    {
        case Connectivity::N4:
            for (const auto& o : ortho2d)  result.push_back(pos + o);
            break;
        case Connectivity::N8:
            for (const auto& o : ortho2d)  result.push_back(pos + o);
            for (const auto& o : diag2d)   result.push_back(pos + o);
            break;
        case Connectivity::N6:
            for (const auto& o : face3d)   result.push_back(pos + o);
            break;
        case Connectivity::N18:
            for (const auto& o : face3d)   result.push_back(pos + o);
            for (const auto& o : edge3d)   result.push_back(pos + o);
            break;
        case Connectivity::N26:
            for (const auto& o : face3d)   result.push_back(pos + o);
            for (const auto& o : edge3d)   result.push_back(pos + o);
            for (const auto& o : corner3d) result.push_back(pos + o);
            break;
    }
    return result;
}

/// 纯几何步长代价：变化 1 个轴 → 1.0，2 个轴 → √2，3 个轴 → √3
inline double calculateStepCost(const Vec3& from, const Vec3& to) noexcept
{
    int nd = (from.x != to.x) + (from.y != to.y) + (from.z != to.z);
    if (nd <= 1) return 1.0;
    if (nd == 2) return std::sqrt(2.0);
    return std::sqrt(3.0);
}

/// 两个栅格之间的欧氏距离
inline double cellDistance(const Vec3& a, const Vec3& b) noexcept
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// =============================================================
// 本文件提供的共享基础设施
//
// 以下组件对 A*、BFS 以及未来新栅格算法均适用：
//
//   • Vec3 / Vec3Hash           — 栅格坐标类型（vec3.hpp）
//   • Connectivity 枚举          — N4/N8/N6/N18/N26
//   • getNeighbors()            — 按连接方式枚举邻居
//   • calculateStepCost()       — 几何步长代价（1 / √2 / √3）
//   • cellDistance()            — 栅格间欧氏距离
//
// 新算法接入时注意事项：
//
//   1. 路径回溯各算法自实现 —— A* 沿 parent 链回溯，D* Lite 沿 g 值梯度下降，
//      RRT* 沿树边回溯，JPS 在跳点间插值。不要试图做"通用回溯"模板——
//      数据结构和语义各不相同，强行统一只会增加耦合。
//
//   2. 只需引入你需要的组件即可：
//        #include "finenav_route_planner/common/grid_search_utils.hpp"
//        using finenav::Connectivity;
//        using finenav::getNeighbors;
//        using finenav::calculateStepCost;
//
//   3. 如果新算法需要额外的通用栅格操作（如射线投射、bresenham 直线等），
//      优先加在此文件中而非在各算法内重复实现。
// =============================================================

} // namespace finenav

