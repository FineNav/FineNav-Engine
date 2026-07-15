// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/route/route_graph.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

namespace route {

namespace {

using Json = nlohmann::json;

/// 计算折线的欧氏总长度（相邻点距离之和，单位：米）
double polylineLength(const std::vector<std::pair<double, double>>& pts)
{
    double len = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        double dx = pts[i].first  - pts[i - 1].first;
        double dy = pts[i].second - pts[i - 1].second;
        len += std::hypot(dx, dy);
    }
    return len;
}

/// 从 GeoJSON geometry 中提取折线点序列（LineString 直接读取，MultiLineString 展平拼接）
std::vector<std::pair<double, double>> collectPolyline(
    const Json& geometry, const std::string& gtype)
{
    std::vector<std::pair<double, double>> pts;
    const auto& coords = geometry["coordinates"];

    if (gtype == "LineString") {
        pts.reserve(coords.size());
        for (const auto& p : coords)
            pts.emplace_back(p[0].get<double>(), p[1].get<double>());
    } else {                         // MultiLineString
        for (const auto& line : coords)
            for (const auto& p : line)
                pts.emplace_back(p[0].get<double>(), p[1].get<double>());
    }
    return pts;
}

}  // anonymous namespace

std::optional<Graph> loadFromJson(const std::string& geojson)
{
    // 解析 JSON 文本，任何异常均视为失败
    Json j;
    try { j = Json::parse(geojson); }
    catch (...) { return std::nullopt; }

    // 必须包含 features 数组（FeatureCollection）
    if (!j.contains("features") || !j["features"].is_array())
        return std::nullopt;

    Graph g;

    // ===== 第一遍扫描：解析所有节点（Point 要素）=====
    for (const auto& feat : j["features"]) {
        if (!feat.contains("geometry")) continue;
        if (feat["geometry"]["type"] != "Point") continue;

        const auto& props  = feat["properties"];
        const auto& coords = feat["geometry"]["coordinates"];
        if (!coords.is_array() || coords.size() < 2) return std::nullopt;
        if (!props.contains("id")) return std::nullopt;

        Node node;
        node.id = props["id"].get<unsigned int>();
        node.x  = coords[0].get<double>();
        node.y  = coords[1].get<double>();
        if (props.contains("frame") && props["frame"].is_string())
            node.frame = props["frame"].get<std::string>();

        g.id_index[node.id] = g.nodes.size();
        g.nodes.push_back(std::move(node));
    }

    // 无任何节点视为无效路网
    if (g.nodes.empty()) return std::nullopt;
    g.adj.resize(g.nodes.size());

    // ===== 第二遍扫描：解析所有边（LineString / MultiLineString 要素）=====
    for (const auto& feat : j["features"]) {
        if (!feat.contains("geometry")) continue;
        const std::string gtype = feat["geometry"]["type"];
        if (gtype != "LineString" && gtype != "MultiLineString") continue;

        const auto& props = feat["properties"];
        if (!props.contains("startid") || !props.contains("endid"))
            return std::nullopt;

        const size_t si = g.indexOf(props["startid"].get<unsigned int>());
        const size_t ei = g.indexOf(props["endid"].get<unsigned int>());
        if (si == SIZE_MAX || ei == SIZE_MAX) return std::nullopt;

        auto polyline = collectPolyline(feat["geometry"], gtype);
        if (polyline.empty()) {
            // 几何为空时退化为起终点节点中心连成的直线段
            polyline.emplace_back(g.nodes[si].x, g.nodes[si].y);
            polyline.emplace_back(g.nodes[ei].x, g.nodes[ei].y);
        }

        // 仅当边被显式标记为不可覆盖（overridable == false）且 cost > 0 时使用固定代价
        const bool fixed_cost =
            props.contains("cost") &&
            props.contains("overridable") &&
            !props["overridable"].get<bool>() &&
            props["cost"].get<double>() > 0.0;

        // 边代价：固定代价 或 折线欧氏长度
        const double cost = fixed_cost
            ? props["cost"].get<double>()
            : polylineLength(polyline);

        g.adj[si].push_back({ei, cost, std::move(polyline)});
    }

    return g;
}

}  // namespace route

