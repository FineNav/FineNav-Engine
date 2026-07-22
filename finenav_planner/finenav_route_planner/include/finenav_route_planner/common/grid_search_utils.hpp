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
// reconstructPathImpl
//
// A* 与 BFS 共享的路径回溯辅助函数。
//
//   ParentMapT : unordered_map<Vec3, NodeT, Vec3Hash>
//   GetParentFn: (const NodeT&) -> Vec3
//
// A* 传入:  came_from_<Vec3, AstarNode>,  [](auto& n){ return n.parent; }
// BFS 传入: parent_   <Vec3, Vec3>,        [](auto& v){ return v; }
// =============================================================

template<typename NodeT, typename GetParentFn>
inline std::vector<Vec3> reconstructPathImpl( // TODO： 之后要想一个语义更好的实现
    const Vec3& goal,
    const std::unordered_map<Vec3, NodeT, Vec3Hash>& parent_map,
    GetParentFn get_parent)
{
    std::vector<Vec3> path;
    Vec3 cur = goal;
    while (true) {
        path.push_back(cur);
        const auto& node   = parent_map.at(cur);
        const Vec3  parent = get_parent(node);
        if (cur == parent) break;   // 自哨兵（parent == 自身）即为起点
        cur = parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace astar

