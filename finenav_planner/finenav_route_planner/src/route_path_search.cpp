// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/route_path_search.hpp"

#include <cmath>
#include <limits>
#include <unordered_map>
#include <mutex>

#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

// ── configure ──────────────────────────────────────────────────────────────
//
// 每当动态参数变化时被调用。
// 仅当 graph_file 实际发生变化时才重新加载路网图。
// TODO: 上一行描述已过时——当前没有 graph_file 参数，路网图通过 /geojson 话题加载，待确认后修正。

void RoutePathSearch::configure(const ConfigType& params)
{
    path_density_             = params.path_density;
    min_approach_angle_deg_   = params.min_approach_angle_deg;
    terrain_weight_           = params.terrain_weight;
}

// ── updateGraph ────────────────────────────────────────────────────────────

void RoutePathSearch::updateGraph(const std::string& geojson_str)
{
    std::lock_guard<std::mutex> lock(*graph_mutex_);
    cached_graph_ = route::loadFromJson(geojson_str);

    if (!cached_graph_) {
        RCLCPP_ERROR(
            rclcpp::get_logger("RoutePathSearch"),
            "Failed to parse route graph from /geojson topic.");
    } else {
        RCLCPP_INFO(
            rclcpp::get_logger("RoutePathSearch"),
            "Successfully loaded route graph from /geojson topic with %zu nodes.",
            cached_graph_->nodes.size());
    }
}

// ── plan ───────────────────────────────────────────────────────────────────

RoutePathSearch::Result RoutePathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    std::lock_guard<std::mutex> lock(*graph_mutex_);

    if (!cached_graph_ || cached_graph_->nodes.empty()) return std::nullopt;
    const auto& orig_graph = *cached_graph_;

    // 避障 BFS：将起点和目标吸附到最近可达的图点
    // （节点或 polyline 中间路径点）。
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = ctx.robot_state.position.x();
    start_pose.position.y = ctx.robot_state.position.y();
    start_pose.position.z = ctx.robot_state.position.z();
    start_pose.orientation.w = ctx.robot_state.orientation.w();
    start_pose.orientation.x = ctx.robot_state.orientation.x();
    start_pose.orientation.y = ctx.robot_state.orientation.y();
    start_pose.orientation.z = ctx.robot_state.orientation.z();
    const auto start_snap = findNearestGraphPoint(orig_graph, start_pose, map_view);
    if (!start_snap.isNode() && !start_snap.isPath()) return std::nullopt;

    const auto goal_snap = findNearestGraphPoint(orig_graph, ctx.goal_pose, map_view);
    if (!goal_snap.isNode() && !goal_snap.isPath()) return std::nullopt;

    // ── 构建可变工作图 ──────────────────────────────────────
    // 对于路径点吸附的出入口，可能需要插入虚拟节点并分裂边。
    // 在副本上操作，使缓存的原图不受影响。
    route::Graph graph = orig_graph;   // 深拷贝

    size_t start_node_idx = SIZE_MAX;
    size_t goal_node_idx  = SIZE_MAX;

    // 辅助函数：将 SnapResult 转换为 `graph` 中的有效节点下标。
    // 节点吸附时原下标仍然有效（我们只做追加操作）。
    // 路径点吸附时插入虚拟节点并分裂该边。
    // TODO: 参数 is_start 从未被使用，待确认是删除还是本应参与逻辑。
    auto resolveSnap = [&](const SnapResult& snap, bool is_start) -> size_t {
        if (snap.isNode()) {
            return snap.node_index;
        }
        // 路径点吸附 — 插入虚拟节点并分裂该边
        const size_t vi = insertVirtualNode(graph, snap.snap_point);
        splitEdgeForSnap(graph, snap, vi);
        return vi;
    };

    // 当两个吸附点落在同一条边上时，按 pt_idx 递减的顺序处理，
    // 以免先前的分裂使后续下标失效。
    // 最简单的做法：先处理起点，再处理目标。splitEdgeForSnap 按
    // to_index 和 polyline 范围进行查找，因此对下标偏移是稳健的。
    start_node_idx = resolveSnap(start_snap, true);
    goal_node_idx  = resolveSnap(goal_snap, false);

    // ── 在（可能已修改的）拓扑图上运行 Dijkstra ────────────
    auto route_opt = route::findRoute(graph, start_node_idx, goal_node_idx, path_density_);
    if (!route_opt || route_opt->empty()) return std::nullopt;

    OutputTrajectoryT<RoutePathSearch> output;
    output.poses.reserve(start_snap.connector_path.size()
                       + goal_snap.connector_path.size()
                       + route_opt->size());

    auto appendPose = [&](double x, double y) {
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    };

    auto isDegenerate = [](const std::vector<Position3D>& conn) -> bool {
        if (conn.size() < 2) return true;
        const double d = (conn.front() - conn.back()).norm();
        return d < 1e-6;
    };

    // ── 起点连接段（robot_pose → snap_point）──
    if (!isDegenerate(start_snap.connector_path)) {
        for (const auto& pt : start_snap.connector_path)
            appendPose(pt.x(), pt.y());
    }

    // ── 拓扑路线 ──
    // 若路线首点与连接段末点重合，则跳过路线首点。
    bool skip_first_route = false;
    if (!isDegenerate(start_snap.connector_path) && !output.poses.empty()) {
        const auto& last_conn = output.poses.back().position;
        const auto& first_route_x = (*route_opt)[0].first;
        const auto& first_route_y = (*route_opt)[0].second;
        skip_first_route = (std::hypot(last_conn.x - first_route_x,
                                        last_conn.y - first_route_y) < 1e-6);
    }

    for (size_t i = (skip_first_route ? 1 : 0); i < route_opt->size(); ++i)
        appendPose((*route_opt)[i].first, (*route_opt)[i].second);

    // ── 目标连接段（snap_point → goal_pose），反向遍历 ──
    if (!isDegenerate(goal_snap.connector_path)) {
        const auto& route_end = output.poses.back().position;
        const auto& goal_last = goal_snap.connector_path.back();  // 吸附点
        const bool route_ends_at_snap =
            std::hypot(route_end.x - goal_last.x(),
                       route_end.y - goal_last.y()) < 1e-6;

        const size_t start_i = route_ends_at_snap
            ? goal_snap.connector_path.size() - 2
            : goal_snap.connector_path.size() - 1;

        for (size_t i = start_i; i != SIZE_MAX; --i)
            appendPose(goal_snap.connector_path[i].x(),
                       goal_snap.connector_path[i].y());
    }

    return output;
}

// ── findNearestGraphPoint ───────────────────────────────────────────────────
//
// 在真实占据地图上运行 AStarAlgorithm::findPathToAny（h=0，即 Dijkstra）。
// 搜索从包含 `pose` 的栅格单元向外扩展，直到找到一个至少包含一个路网图节点
// 或某条边 polyline 中间点（即"路径点"）的栅格单元为止。
//
// 使用真实地图意味着连接段是可避障的：只能穿墙才能到达的点不会被选中。
//
// 栅格尺寸 = map_view.getStepSize()（地图分辨率），因此该 BFS 与
// AstarPathSearch / BfsPathSearch 处于同一栅格空间。
//
// 返回 SnapResult，包含入口点信息以及从 `pose` 到该入口点的世界坐标系连接路径。

namespace {
struct PathPointInfo {
    size_t                from_node;
    size_t                to_node;
    size_t                edge_idx;   // 在 graph.adj[from_node] 中的下标
    size_t                pt_idx;     // 在该边 polyline 中的下标
    finenav::Position3D   world_pos;
};
}  // 匿名命名空间

RoutePathSearch::SnapResult RoutePathSearch::findNearestGraphPoint(
    const route::Graph&             graph,
    const geometry_msgs::msg::Pose& pose,
    const IMapView&                 map_view) const
{
    const double step = map_view.getStepSize();

    auto toCell = [step](double x, double y) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            0
        };
    };
    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step, (c.y + 0.5) * step, 0.0 };
    };

    const astar::Vec3 origin = toCell(pose.position.x, pose.position.y);

    // 构建 cell → 节点下标 映射
    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_nodes;
    int max_depth = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        const astar::Vec3 c = toCell(graph.nodes[i].x, graph.nodes[i].y);
        cell_nodes[c].push_back(i);
        max_depth = std::max(max_depth,
            std::max(std::abs(c.x - origin.x), std::abs(c.y - origin.y)));
    }

    // 由 polyline 中间点构建 cell → 路径点信息 映射。
    // 跳过首点（0）和末点（size-1）— 它们与节点重合。
    std::unordered_map<astar::Vec3, std::vector<PathPointInfo>, astar::Vec3Hash> cell_path_pts;
    for (size_t fi = 0; fi < graph.adj.size(); ++fi) {
        for (size_t ei = 0; ei < graph.adj[fi].size(); ++ei) {
            const auto& pl = graph.adj[fi][ei].polyline;
            if (pl.size() <= 2) continue;  // 没有中间点
            const size_t to_idx = graph.adj[fi][ei].to_index;
            for (size_t pi = 1; pi + 1 < pl.size(); ++pi) {
                const astar::Vec3 c = toCell(pl[pi].first, pl[pi].second);
                cell_path_pts[c].push_back({
                    fi, to_idx, ei, pi,
                    {pl[pi].first, pl[pi].second, 0.0}
                });
                max_depth = std::max(max_depth,
                    std::max(std::abs(c.x - origin.x), std::abs(c.y - origin.y)));
            }
        }
    }

    // 避障 lambda — 使用真实地图数据
    auto is_walkable = [&](const astar::Vec3& c) {
        return c.z == 0 &&
               std::abs(c.x - origin.x) <= max_depth &&
               std::abs(c.y - origin.y) <= max_depth &&
               map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) {
        return map_view.getCost(cellCentre(c));
    };
    auto is_goal = [&](const astar::Vec3& c) {
        return cell_nodes.contains(c) || cell_path_pts.contains(c);
    };

    // ── 路径点吸附的角度过滤 ─────────────────────────────
    // 若某个目标单元内所有路径点候选的接近角都小于 min_approach_angle_deg_，
    // 则拒绝该单元，使 Dijkstra 越过它继续向前搜索。
    // 节点总是被接受（节点不做角度检查）。
    astar::AStarAlgorithm::GoalFilterFn accept_goal = nullptr;
    if (min_approach_angle_deg_ > 0.0) {
        accept_goal = [&](const astar::Vec3& c) -> bool {
            // 节点总是可接受
            if (cell_nodes.contains(c)) return true;
            // 对路径点单元，检查是否有任一候选通过
            if (cell_path_pts.contains(c)) {
                for (const auto& ppi : cell_path_pts.at(c)) {
                    // 接近方向：机器人位姿 → 候选点
                    const double adx = ppi.world_pos.x() - pose.position.x;
                    const double ady = ppi.world_pos.y() - pose.position.y;
                    const double ad_norm = std::hypot(adx, ady);
                    if (ad_norm < 1e-9) continue;
                    // 路径切向：polyline[pt+1] − polyline[pt−1]
                    const auto& pl = graph.adj[ppi.from_node][ppi.edge_idx].polyline;
                    const double tdx = pl[ppi.pt_idx + 1].first  - pl[ppi.pt_idx - 1].first;
                    const double tdy = pl[ppi.pt_idx + 1].second - pl[ppi.pt_idx - 1].second;
                    const double td_norm = std::hypot(tdx, tdy);
                    if (td_norm < 1e-9) continue;
                    const double cos_angle = (adx * tdx + ady * tdy) / (ad_norm * td_norm);
                    const double angle_deg =
                        std::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / M_PI;
                    if (angle_deg >= min_approach_angle_deg_) return true;
                }
                return false;   // 该单元内没有路径点通过检查
            }
            return false;   // 既不是节点也不是路径点（不应发生）
        };
    }

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::Connectivity::N8);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setMaxIterations(-1);
    algo.setMaxPlanningTime(5.0); // TODO: 后面这些应该也成为动态参数
    algo.setTerrainWeight(terrain_weight_);

    const auto path = algo.findPathToAny(origin, is_goal, accept_goal);
    if (!path || path->empty()) return {};

    const astar::Vec3& goal_cell = path->back();

    SnapResult result;

    // ── 在节点和路径点中寻找几何距离最近的候选 ──
    double best_dist = std::numeric_limits<double>::infinity();
    bool   found_node = false;

    // 检查目标单元内的节点
    if (cell_nodes.contains(goal_cell)) {
        for (const size_t idx : cell_nodes.at(goal_cell)) {
            const double d = std::hypot(graph.nodes[idx].x - pose.position.x,
                                        graph.nodes[idx].y - pose.position.y);
            if (d < best_dist ||
                (std::abs(d - best_dist) < 1e-9 &&
                 graph.nodes[idx].id < graph.nodes[result.node_index].id))
            {
                result.node_index = idx;
                best_dist = d;
                found_node = true;
            }
        }
    }

    // 检查目标单元内的路径点
    if (cell_path_pts.contains(goal_cell)) {
        for (const auto& ppi : cell_path_pts.at(goal_cell)) {
            // ── 角度过滤：跳过接近角过小的路径点 ──
            if (min_approach_angle_deg_ > 0.0) {
                const double adx = ppi.world_pos.x() - pose.position.x;
                const double ady = ppi.world_pos.y() - pose.position.y;
                const double ad_norm = std::hypot(adx, ady);
                if (ad_norm > 1e-9) {
                    const auto& pl = graph.adj[ppi.from_node][ppi.edge_idx].polyline;
                    const double tdx = pl[ppi.pt_idx + 1].first  - pl[ppi.pt_idx - 1].first;
                    const double tdy = pl[ppi.pt_idx + 1].second - pl[ppi.pt_idx - 1].second;
                    const double td_norm = std::hypot(tdx, tdy);
                    if (td_norm > 1e-9) {
                        const double cos_angle = (adx * tdx + ady * tdy) / (ad_norm * td_norm);
                        const double angle_deg =
                            std::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / M_PI;
                        if (angle_deg < min_approach_angle_deg_) continue;  // 跳过该候选
                    }
                }
            }

            const double d = std::hypot(ppi.world_pos.x() - pose.position.x,
                                        ppi.world_pos.y() - pose.position.y);
            if (d < best_dist) {
                // 若从节点切换到路径点，重置 node_index
                if (found_node) {
                    result.node_index = SIZE_MAX;
                    found_node = false;
                }
                result.edge_from  = ppi.from_node;
                result.edge_to    = ppi.to_node;
                result.edge_idx   = ppi.edge_idx;
                result.pt_idx     = ppi.pt_idx;
                result.snap_point = ppi.world_pos;
                best_dist = d;
            }
        }
    }

    // ── 构建世界坐标系连接路径（pose → snap_point）──────────
    std::vector<Position3D> connector;
    connector.reserve(path->size() + 1);

    // 首点：精确的连续位姿
    connector.push_back({pose.position.x, pose.position.y, 0.0});

    // 中间栅格单元 → 单元中心（若 origin 与首点重合则跳过）
    const bool skip_first = (path->size() >= 2 && (*path)[0] == origin);
    for (size_t i = (skip_first ? 1 : 0); i < path->size(); ++i) {
        connector.push_back(cellCentre((*path)[i]));
    }

    // 末点：替换为精确的吸附点位置
    if (result.isNode()) {
        connector.back() = {graph.nodes[result.node_index].x,
                           graph.nodes[result.node_index].y, 0.0};
    } else {
        connector.back() = result.snap_point;
    }

    result.connector_path = std::move(connector);
    return result;
}

// ── insertVirtualNode ───────────────────────────────────────────────────────
//
// 追加一个没有 id 的虚拟节点（id_index 保持不动）。
// 返回新节点在 graph.nodes 中的下标。

size_t RoutePathSearch::insertVirtualNode(
    route::Graph&     graph,
    const Position3D& pos) const
{
    const size_t idx = graph.nodes.size();
    graph.nodes.push_back({/*id=*/0, pos.x(), pos.y(), /*frame=*/""});
    graph.adj.resize(graph.nodes.size());
    return idx;
}

// ── splitEdgeForSnap ────────────────────────────────────────────────────────
//
// 在 graph.adj 中查找任意一条其 polyline 包含与 snap.snap_point 匹配点的有向边，
// 然后将其替换为两条子边：
//   src → virtual_idx   （polyline 截至并包含吸附点）
//   virtual_idx → dst   （polyline 从吸附点起往后）
//
// 若存在反向边（dst → src），则以同样方式分裂。
//
// 会遍历所有邻接表 — 对同一原始边先前的分裂（可能已将目标 polyline
// 移动到某个虚拟节点上）具有稳健性。

void RoutePathSearch::splitEdgeForSnap(
    route::Graph&     graph,
    const SnapResult& snap,
    size_t            virtual_idx) const
{
    const double kTol = 1e-6;

    // ── 查找并分裂正向边（包含 snap_point 的那条边）──
    bool found = false;
    for (size_t si = 0; si < graph.adj.size() && !found; ++si) {
        for (size_t ei = 0; ei < graph.adj[si].size() && !found; ++ei) {
            auto& nb = graph.adj[si][ei];
            if (nb.polyline.size() < 2) continue;

            // 在该 polyline 中查找吸附点
            for (size_t pi = 0; pi < nb.polyline.size(); ++pi) {
                const double dx = nb.polyline[pi].first  - snap.snap_point.x();
                const double dy = nb.polyline[pi].second - snap.snap_point.y();
                if (std::hypot(dx, dy) > kTol) continue;

                // 找到了 — 在下标 pi 处分裂
                const size_t src = si;
                const size_t dst = nb.to_index;

                // ── 子 polyline ──
                std::vector<std::pair<double, double>> pl_first(
                    nb.polyline.begin(), nb.polyline.begin() + pi + 1);
                std::vector<std::pair<double, double>> pl_second(
                    nb.polyline.begin() + pi, nb.polyline.end());

                // ── 按长度比例分配代价 ──
                double len_first = 0.0, len_second = 0.0;
                for (size_t i = 1; i < pl_first.size(); ++i)
                    len_first += std::hypot(pl_first[i].first - pl_first[i-1].first,
                                            pl_first[i].second - pl_first[i-1].second);
                for (size_t i = 1; i < pl_second.size(); ++i)
                    len_second += std::hypot(pl_second[i].first - pl_second[i-1].first,
                                             pl_second[i].second - pl_second[i-1].second);
                const double total_len = len_first + len_second;
                const double cost_first  = (total_len > 1e-9)
                    ? nb.cost * (len_first / total_len) : nb.cost * 0.5;
                const double cost_second = (total_len > 1e-9)
                    ? nb.cost * (len_second / total_len) : nb.cost * 0.5;

                // 用第一条子边（src → virtual）替换原边
                nb.to_index = virtual_idx;
                nb.polyline = std::move(pl_first);
                nb.cost     = cost_first;

                // 添加第二条子边（virtual → dst）
                graph.adj[virtual_idx].push_back(
                    {dst, cost_second, std::move(pl_second)});

                found = true;

                // ── 若存在反向边（dst → src）则一并分裂 ──
                for (auto& rev_nb : graph.adj[dst]) {
                    if (rev_nb.to_index != src) continue;
                    if (rev_nb.polyline.size() < 2) continue;

                    // 在反向 polyline 中查找吸附点
                    for (size_t rpi = 0; rpi < rev_nb.polyline.size(); ++rpi) {
                        const double rdx = rev_nb.polyline[rpi].first  - snap.snap_point.x();
                        const double rdy = rev_nb.polyline[rpi].second - snap.snap_point.y();
                        if (std::hypot(rdx, rdy) > kTol) continue;

                        std::vector<std::pair<double, double>> rpl_first(
                            rev_nb.polyline.begin(), rev_nb.polyline.begin() + rpi + 1);
                        std::vector<std::pair<double, double>> rpl_second(
                            rev_nb.polyline.begin() + rpi, rev_nb.polyline.end());

                        double rlen_first = 0.0, rlen_second = 0.0;
                        for (size_t i = 1; i < rpl_first.size(); ++i)
                            rlen_first += std::hypot(rpl_first[i].first - rpl_first[i-1].first,
                                                     rpl_first[i].second - rpl_first[i-1].second);
                        for (size_t i = 1; i < rpl_second.size(); ++i)
                            rlen_second += std::hypot(rpl_second[i].first - rpl_second[i-1].first,
                                                      rpl_second[i].second - rpl_second[i-1].second);
                        const double rtotal = rlen_first + rlen_second;
                        const double rcost_first  = (rtotal > 1e-9)
                            ? rev_nb.cost * (rlen_first / rtotal) : rev_nb.cost * 0.5;
                        const double rcost_second = (rtotal > 1e-9)
                            ? rev_nb.cost * (rlen_second / rtotal) : rev_nb.cost * 0.5;

                        rev_nb.to_index = virtual_idx;
                        rev_nb.polyline = std::move(rpl_first);
                        rev_nb.cost     = rcost_first;

                        graph.adj[virtual_idx].push_back(
                            {src, rcost_second, std::move(rpl_second)});
                        break;
                    }
                    break;
                }
                break;
            }
        }
    }
}

} // namespace finenav

