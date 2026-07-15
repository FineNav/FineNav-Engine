// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/route/route_search.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace route {

namespace {

/**
 * @brief 将 (x0,y0) 到 (x1,y1) 线段按步长 density 插值，并把插值点追加到 out
 * @details 起点仅在与 out 中最后一点不同（距离 > 1e-9）时才追加，避免相邻线段产生重复点。
 * @note 本函数不追加线段终点 (x1,y1)：终点由下一段的起点补上，最终目标点由调用方补齐。
 */
void densifySegment(
    double x0, double y0, double x1, double y1,
    double density,
    RoutePoses& out)
{
    // 去重追加：仅当与 out 末尾点距离 > 1e-9 时才加入新点
    auto append = [&](double x, double y) {
        if (out.empty() || std::hypot(out.back().first - x, out.back().second - y) > 1e-9)
            out.emplace_back(x, y);
    };

    const double dist = std::hypot(x1 - x0, y1 - y0);
    if (dist < 1e-9) { append(x0, y0); return; }   // 零长度线段：仅追加起点

    const int    n    = std::max(1, static_cast<int>(std::ceil(dist / density)));
    const double step = dist / n;
    const double ux   = (x1 - x0) / dist;
    const double uy   = (y1 - y0) / dist;

    // 沿单位方向 (ux, uy) 等距生成 n 个插值点（含起点，不含终点）
    for (int i = 0; i < n; ++i)
        append(x0 + ux * step * i, y0 + uy * step * i);
}

}  // anonymous namespace

std::optional<RoutePoses> findRoute(
    const Graph& graph,
    size_t       start_index,
    size_t       goal_index,
    double       path_density)
{
    // 起终点下标越界检查
    const size_t N = graph.nodes.size();
    if (start_index >= N || goal_index >= N) return std::nullopt;

    // 起终点相同：直接返回单点路径
    if (start_index == goal_index)
        return RoutePoses{{graph.nodes[start_index].x, graph.nodes[start_index].y}};

    // ===== Dijkstra 最短路搜索 =====
    constexpr double INF = std::numeric_limits<double>::infinity();
    std::vector<double> dist(N, INF);
    std::vector<size_t> prev(N, SIZE_MAX);
    std::vector<size_t> prev_edge(N, SIZE_MAX);   // 到达该节点所经边在 adj[prev] 中的下标

    using QEntry = std::pair<double, size_t>;     // (累计代价, 节点下标)
    std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> pq;

    dist[start_index] = 0.0;
    pq.emplace(0.0, start_index);

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;      // 过期队列条目（惰性删除），跳过
        if (u == goal_index) break;     // 目标节点已弹出，最短路已确定，提前结束

        // 松弛 u 的所有出边
        for (size_t k = 0; k < graph.adj[u].size(); ++k) {
            const Neighbor& nb = graph.adj[u][k];
            const double nd = d + nb.cost;
            if (nd < dist[nb.to_index]) {
                dist[nb.to_index] = nd;
                prev[nb.to_index]      = u;
                prev_edge[nb.to_index] = k;
                pq.emplace(nd, nb.to_index);
            }
        }
    }

    // 目标不可达（图不连通）
    if (dist[goal_index] == INF) return std::nullopt;

    // ===== 回溯边序列（正向：从起点到终点）=====
    // edges_fwd[i] = (边的起点节点下标 from, 该边在 adj[from] 中的下标)
    std::vector<std::pair<size_t, size_t>> edges_fwd;
    for (size_t cur = goal_index; cur != start_index; ) {
        size_t p = prev[cur];
        edges_fwd.emplace_back(p, prev_edge[cur]);
        cur = p;
    }
    std::reverse(edges_fwd.begin(), edges_fwd.end());

    // ===== 构建输出路径 =====
    RoutePoses result;

    if (path_density <= 0.0) {
        // 仅返回沿途节点中心点（不做插值加密）
        result.emplace_back(graph.nodes[start_index].x, graph.nodes[start_index].y);
        for (auto [fi, k] : edges_fwd) {
            const size_t ti = graph.adj[fi][k].to_index;
            result.emplace_back(graph.nodes[ti].x, graph.nodes[ti].y);
        }
        return result;
    }

    // 对每条边的折线逐段插值加密
    for (auto [fi, k] : edges_fwd) {
        const auto& pl = graph.adj[fi][k].polyline;
        for (size_t i = 0; i + 1 < pl.size(); ++i)
            densifySegment(pl[i].first, pl[i].second,
                           pl[i+1].first, pl[i+1].second,
                           path_density, result);
    }

    // 保证精确的目标点是路径的最后一个点
    const auto& gn = graph.nodes[goal_index];
    if (result.empty() ||
        std::hypot(result.back().first - gn.x, result.back().second - gn.y) > 1e-9)
        result.emplace_back(gn.x, gn.y);

    return result;
}

}  // namespace route

