// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <vector>

#include "finenav_route_planner/route/route_graph.hpp"

namespace route {

// ===== 路网最短路搜索 =====

/// 路径点序列：世界坐标系下的 [x, y] 点对列表
using RoutePoses = std::vector<std::pair<double, double>>;

/**
 * @brief 在路网图上执行 Dijkstra 最短路搜索，并对边折线插值加密生成稠密路径点序列
 * @param graph 由 loadFromJson() 构建的路网图
 * @param start_index 起点节点在 graph.nodes 中的下标
 * @param goal_index 终点节点在 graph.nodes 中的下标
 * @param path_density 插值步长（米）；传 0（或负数）时仅返回沿途节点中心点
 * @return 沿路径的世界坐标系 [x, y] 点序列；无可达路径（图不连通）时返回 std::nullopt
 */
std::optional<RoutePoses> findRoute(
    const Graph& graph,
    size_t       start_index,
    size_t       goal_index,
    double       path_density = 0.05);

}  // namespace route

