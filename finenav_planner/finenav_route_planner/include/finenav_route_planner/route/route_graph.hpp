// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace route {
/**
 * @note 当前路网图仅支持 2D（Node / polyline 仅含 x,y 坐标）。
 *       推广到 3D 需逐层改动：
 *       1. Node 增加 z 字段，polyline 增加第三维
 *       2. loadFromJson 读取 coordinates 中的 z
 *       3. findRoute (Dijkstra) 的代价计算考虑 z 轴距离
 *       4. insertVirtualNode / splitEdgeForSnap 处理 z 维度
 */

// ===== 图数据类型 =====

/**
 * @brief 路网图中的节点——世界坐标系下的具名路标点
 */
struct Node {
    unsigned int id    = 0;     ///< 节点唯一 id（来自 GeoJSON properties.id）
    double       x     = 0.0;   ///< 世界坐标系 x（米）
    double       y     = 0.0;   ///< 世界坐标系 y（米）
    std::string  frame;          ///< 语义名称（如 "entrance_A"）；未设置时为空
};

/**
 * @brief 邻接表中的一条有向边
 */
struct Neighbor {
    size_t to_index = 0;         ///< 终点节点在 Graph::nodes 中的下标
    double cost     = 0.0;       ///< 通行代价（米，或固定覆盖值）

    // 稠密折线 [x, y]：首点 = 起点节点中心，末点 = 终点节点中心。
    // 中间点来自 GeoJSON LineString 几何。
    std::vector<std::pair<double, double>> polyline;
};

/**
 * @brief 从 GeoJSON 加载的邻接表路网图
 */
struct Graph {
    std::vector<Node>                        nodes;     ///< 全部节点
    std::vector<std::vector<Neighbor>>       adj;       ///< adj[i] = nodes[i] 的出边列表
    std::unordered_map<unsigned int, size_t> id_index;  ///< 节点 id → 在 nodes 中的下标

    /// 按节点 id 查找下标；未找到返回 SIZE_MAX
    size_t indexOf(unsigned int node_id) const {
        auto it = id_index.find(node_id);
        return (it != id_index.end()) ? it->second : SIZE_MAX;
    }

    /// 按语义 frame 名称查找下标；未找到返回 SIZE_MAX
    size_t indexOfFrame(const std::string& frame_name) const {
        for (size_t i = 0; i < nodes.size(); ++i)
            if (nodes[i].frame == frame_name) return i;
        return SIZE_MAX;
    }
};

// ===== GeoJSON 加载器 =====

/**
 * @brief 从 GeoJSON 文本解析并构建路网图
 * @param geojson GeoJSON FeatureCollection 字符串
 * @return 解析成功时返回 Graph；出现任何解析或 schema 错误时返回 std::nullopt
 * @details 期望的 FeatureCollection schema：
 *          - 节点（Node）：geometry.type = "Point"
 *            properties: { id: uint, frame?: string }
 *            geometry.coordinates: [x, y]
 *          - 边（Edge）：geometry.type = "LineString" | "MultiLineString"
 *            properties: { id: uint, startid: uint, endid: uint,
 *                          cost?: float, overridable?: bool }
 *            geometry.coordinates: [[x,y], ...]
 * @note 若 overridable == false 且 cost > 0，则直接使用该 cost 作为边代价；
 *       否则边代价等于折线的欧氏长度。
 */
std::optional<Graph> loadFromJson(const std::string& geojson);

}  // namespace route

