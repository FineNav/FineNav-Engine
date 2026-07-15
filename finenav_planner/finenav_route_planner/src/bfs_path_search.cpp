// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/bfs_path_search.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::OutputTrajectoryT;
namespace finenav {

// ===== 配置 =====

void BfsPathSearch::configure(const ConfigType& params)
{
    // ── 解析连接方式字符串 → Connectivity 枚举 ──
    if      (params.connectivity == "4")  connectivity_ = astar::Connectivity::N4;
    else if (params.connectivity == "8")  connectivity_ = astar::Connectivity::N8;
    else if (params.connectivity == "6")  connectivity_ = astar::Connectivity::N6;
    else if (params.connectivity == "18") connectivity_ = astar::Connectivity::N18;
    else                                  connectivity_ = astar::Connectivity::N26;

    // ── 读取数值参数 ──
    goal_tolerance_m_  = params.goal_tolerance;
    max_iterations_    = params.max_iterations;
    max_planning_time_ = params.max_planning_time;
    movement_weight_   = params.movement_weight;
    terrain_weight_    = params.terrain_weight;
}

// ===== 规划 =====

BfsPathSearch::Result BfsPathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    const double step = map_view.getStepSize();
    if (step <= 0.0 || !std::isfinite(step))   // 步长非法，无法离散化
        return std::nullopt;

    // ── 世界坐标 → 栅格坐标转换 ──
    auto worldToCell = [step](double x, double y, double z) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            static_cast<int>(std::floor(z / step))
        };
    };

    // ── 栅格坐标 → 世界坐标（栅格中心）──
    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step,
                 (c.y + 0.5) * step,
                 (c.z + 0.5) * step };
    };

    // ── 构建回调函数（栅格坐标 → 世界坐标 → 地图查询）──
    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) -> double {
        return map_view.getCost(cellCentre(c));
    };

    // ── 提取起点/目标栅格坐标 ──
    const auto& sp = ctx.robot_state.position;
    const auto& gp = ctx.goal_pose.position;

    const astar::Vec3 start = worldToCell(sp.x(), sp.y(), sp.z());
    const astar::Vec3 goal  = worldToCell(gp.x, gp.y, gp.z);

    // ── 构建搜索引擎并配置参数 ──
    // BFS 即启发式为 Zero 的 A*（h=0 → 纯代价驱动的 Dijkstra 扩展）
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(connectivity_);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setGoalTolerance(goal_tolerance_m_ / step);
    algo.setMaxIterations(max_iterations_);
    algo.setMaxPlanningTime(max_planning_time_);
    algo.setMovementWeight(movement_weight_);
    algo.setTerrainWeight(terrain_weight_);
    // greedy_weight：h=0 时无影响，保持默认值即可（1.001 * 0 = 0）

    // ── 执行搜索 ──
    auto cell_path = algo.findPath(start, goal);
    if (!cell_path)
        return std::nullopt;

    // ── 将栅格路径转换为世界坐标系位姿序列 ──
    OutputTrajectoryT<BfsPathSearch> output;
    output.poses.reserve(cell_path->size());

    for (const auto& cell : *cell_path) {
        const Position3D w = cellCentre(cell);
        geometry_msgs::msg::Pose p;
        p.position.x    = w.x();
        p.position.y    = w.y();
        p.position.z    = w.z();
        p.orientation.w = 1.0;  // 朝向无约束
        output.poses.push_back(p);
    }

    return output;
}

} // namespace finenav

