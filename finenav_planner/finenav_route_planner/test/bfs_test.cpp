// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file bfs_test.cpp
 * @brief BFS / Dijkstra 模式单元测试 — AStarAlgorithm(Heuristic::Zero) + BfsPathSearch 集成
 *
 * @details
 * ==========================================================================
 * 测试总览
 * ==========================================================================
 * BFS 在代码重构后不再作为独立算法类存在, 而是 AStarAlgorithm 在
 * Heuristic::Zero 配置下的退化形式 (= Dijkstra, 等价于均匀代价网格上的 BFS)。
 * 因此本文件所有测试都通过统一的 AStarAlgorithm + Heuristic::Zero 运行。
 *
 * 被测对象:
 *   - AStarAlgorithm (Heuristic::Zero, h=0 → Dijkstra)
 *   - BfsPathSearch (finenav:: 命名空间, PlannerConcept 实现)
 *   - findPathToAny (BFS 特有的多目标搜索接口)
 *
 * 测试分层:
 *   -# 算法层 — 基础寻路、路径长度、对角线、障碍规避、边界条件、
 *      多目标搜索 (findPathToAny)、3D、带权代价偏好
 *   -# 集成层 — BfsPathSearch + TestMapView, plan 成功/失败/坐标验证
 *
 * ==========================================================================
 * 测试环境
 * ==========================================================================
 * 测试框架:     Google Test (GTest)
 * 依赖:         finenav_route_planner (AStarAlgorithm, BfsPathSearch),
 *               common/grid_map.hpp
 * ROS 依赖:     无（算法层）, 集成层需 ROS 2 头文件
 * 构建:         colcon build --packages-select finenav_route_planner
 *
 * 运行:
 * @code
 *   cd ~/FineNav_ws
 *   colcon test --packages-select finenav_route_planner
 *   # 或直接运行:
 *   ./build/finenav_route_planner/test/bfs_test
 *   # 仅算法:
 *   ./build/finenav_route_planner/test/bfs_test --gtest_filter="BfsAlgorithm2D.*"
 *   # 仅 findPathToAny:
 *   ./build/finenav_route_planner/test/bfs_test --gtest_filter="*FindPathToAny*"
 *   # 仅集成:
 *   ./build/finenav_route_planner/test/bfs_test --gtest_filter="BfsPathSearch.*"
 * @endcode
 *
 * ==========================================================================
 * 测试详细说明
 * ==========================================================================
 *
 * --- 算法层 (BfsAlgorithm2D / BfsAlgorithm3D) ---
 *
 * FindsPath_OpenSpace              — 20×20 空网格, N4, (0,0)→(5,5) 可到达
 * PathLength_Orthogonal            — 直线 5 步 = 6 cells (含起点)
 * Diagonal_AllowedReducesPathLength — N8 路径 < N4 路径
 * PathAvoidsObstacles              — x=3 垂直墙, 路径不穿越
 * StartEqualsGoal                  — 单 cell 路径
 * GoalBlocked_ReturnsNullopt       — 终点被占据 → nullopt
 * MaxIterations_TerminatesEarly    — 1000² 网格, max_iterations=10 → 不挂死
 * MaxIterationsZero_ReturnsNulloptForDifferentGoal — max_iterations=0, 起点≠终点 → nullopt
 *
 * findPathToAny (BFS/Dijkstra 特有的多目标搜索):
 *   FindPathToAny_ReturnsStartWhenStartIsGoal         — 起点满足 goal_pred → 立即返回
 *   FindPathToAny_StopsAtFirstReachableGoal           — 两个候选目标 (3,0) 和 (8,0),
 *                                                       算法停在更近的 (3,0)
 *
 * 带权代价:
 *   PrefersLowerCostPath_WeightedGrid — x=2 列有 3 个高代价 cell (cost=100),
 *                                         Dijkstra 应绕道低代价路径而非直线穿过
 *
 * 3D:
 *   FindsPath_OpenVolume             — (0,0,0)→(3,3,3), N26, h=0
 *
 * --- 集成层 (BfsPathSearch) ---
 *
 * PlanSucceeds_OpenGrid             — 10×10, goal_tolerance=0.6, step=1.0
 * PlanFails_GoalBlocked              — 终点占据 → nullopt
 * PlanFails_InvalidStepSize          — step_size=0 → nullopt
 * OutputPoses_AreWorldCoordinates    — 所有输出位姿在 grid 范围内
 *
 * ==========================================================================
 * 设计说明
 * ==========================================================================
 *   - Heuristic::Zero 将 A* 退化为 Dijkstra:
 *     f(n) = g(n) + ε·h(n) 中 h=0 → f(n)=g(n)
 *     此时算法按照实际代价 g(n) 逐层扩展, 等价于 BFS 在均匀代价网格上的行为
 *   - findPathToAny 是 BFS 场景下的特有接口:
 *     接受一个 goal predicate 函数而非单一目标坐标,
 *     在搜索过程中遇到第一个满足条件的 cell 即返回,
 *     用于 RoutePathSearch 中的"找最近可达路由节点"场景
 *   - 带权代价测试 (PrefersLowerCostPath_WeightedGrid) 验证:
 *     Dijkstra 的 g(n) 包含地形代价, 因此会选择累积代价更低的绕道路径,
 *     而不是几何最短但代价更高的直线路径
 *   - 与 A* 测试的关系:
 *     bfs_test 专注于 h=0 的行为 (Dijkstra 模式),
 *     astar_test 专注于 h>0 的行为 (启发式搜索);
 *     两者共享 AStarAlgorithm 代码但验证不同的配置路径
 *
 * @author Jinghui Wang (wangjinghui@sjtu.edu.cn)
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-07-19
 */

#include <gtest/gtest.h>
#include <cmath>

#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/bfs_path_search.hpp"
#include "common/grid_map.hpp"

// ── helpers ──────────────────────────────────────────────────────────────

// Build a Dijkstra-mode (h=0) AStarAlgorithm over a GridMap.
inline astar::AStarAlgorithm makeDijkstra(const finenav::GridMap& g)
{
    auto is_walkable = [g](const finenav::Vec3& c) {
        return g.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [g](const finenav::Vec3& c) {
        return g.getCostCell(c.x, c.y);
    };
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setHeuristic(finenav::Heuristic::Zero);
    return algo;
}

// ── Algorithm tests (via AStarAlgorithm + Heuristic::Zero) ──────────────

TEST(BfsAlgorithm2D, FindsPath_OpenSpace)
{
    auto algo = makeDijkstra(finenav::GridMap(20, 20));
    algo.setConnectivity(finenav::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (finenav::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (finenav::Vec3{5, 5, 0}));
}

TEST(BfsAlgorithm2D, PathLength_Orthogonal)
{
    auto algo = makeDijkstra(finenav::GridMap(20, 20));
    algo.setConnectivity(finenav::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 0, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 6u);
}

TEST(BfsAlgorithm2D, Diagonal_AllowedReducesPathLength)
{
    finenav::GridMap grid(20, 20);
    auto ortho = makeDijkstra(grid);
    ortho.setConnectivity(finenav::Connectivity::N4);
    auto path_ortho = ortho.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_ortho.has_value());

    auto diag = makeDijkstra(grid);
    diag.setConnectivity(finenav::Connectivity::N8);
    auto path_diag = diag.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_diag.has_value());

    EXPECT_LT(path_diag->size(), path_ortho->size());
}

TEST(BfsAlgorithm2D, PathAvoidsObstacles)
{
    finenav::GridMap grid(10, 10);
    for (int y = 0; y <= 8; ++y) grid.setObstacle(3, y, true);

    auto algo = makeDijkstra(grid);
    algo.setConnectivity(finenav::Connectivity::N4);

    auto path = algo.findPath({0, 5, 0}, {6, 5, 0});
    ASSERT_TRUE(path.has_value());
    for (const auto& c : *path) {
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
    }
}

TEST(BfsAlgorithm2D, StartEqualsGoal)
{
    auto algo = makeDijkstra(finenav::GridMap(10, 10));
    auto path = algo.findPath({3, 3, 0}, {3, 3, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (finenav::Vec3{3, 3, 0}));
}

TEST(BfsAlgorithm2D, GoalBlocked_ReturnsNullopt)
{
    finenav::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    auto algo = makeDijkstra(grid);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(BfsAlgorithm2D, MaxIterations_TerminatesEarly)
{
    auto algo = makeDijkstra(finenav::GridMap(1000, 1000));
    algo.setMaxIterations(10);
    algo.setGoalTolerance(0.0);
    (void)algo.findPath({0, 0, 0}, {999, 999, 0});
    SUCCEED();
}

TEST(BfsAlgorithm2D, MaxIterationsZero_ReturnsNulloptForDifferentGoal)
{
    auto algo = makeDijkstra(finenav::GridMap(10, 10));
    algo.setMaxIterations(0);
    algo.setGoalTolerance(0.0);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {1, 0, 0}).has_value());
}

// findPathToAny is now a method of AStarAlgorithm (h forced to 0 internally)
TEST(BfsAlgorithm2D, FindPathToAny_ReturnsStartWhenStartIsGoal)
{
    auto algo = makeDijkstra(finenav::GridMap(10, 10));
    auto path = algo.findPathToAny({3, 3, 0}, [](const finenav::Vec3& c) {
        return c == finenav::Vec3{3, 3, 0};
    });
    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (finenav::Vec3{3, 3, 0}));
}

TEST(BfsAlgorithm2D, FindPathToAny_StopsAtFirstReachableGoal)
{
    auto algo = makeDijkstra(finenav::GridMap(20, 20));
    algo.setConnectivity(finenav::Connectivity::N8);

    auto path = algo.findPathToAny({0, 0, 0}, [](const finenav::Vec3& c) {
        return c == finenav::Vec3{3, 0, 0} || c == finenav::Vec3{8, 0, 0};
    });
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->back(), (finenav::Vec3{3, 0, 0}));
    EXPECT_EQ(path->size(), 4u);
}

TEST(BfsAlgorithm3D, FindsPath_OpenVolume)
{
    auto is_walkable = [](const finenav::Vec3&) { return true; };
    auto get_cost    = [](const finenav::Vec3&) { return 0.0; };
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setHeuristic(finenav::Heuristic::Zero);
    algo.setConnectivity(finenav::Connectivity::N26);

    auto path = algo.findPath({0, 0, 0}, {3, 3, 3});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (finenav::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (finenav::Vec3{3, 3, 3}));
}

// Weighted-cost test: Dijkstra must prefer low-cost detour over shorter obstacle-adjacent path
TEST(BfsAlgorithm2D, PrefersLowerCostPath_WeightedGrid)
{
    // 5-wide grid. Direct path (x=2 column) has high cost cells.
    // Detour (x=0 column) is longer but cheaper.
    finenav::GridMap grid(5, 6);
    grid.setCost(2, 1, 100.0);
    grid.setCost(2, 2, 100.0);
    grid.setCost(2, 3, 100.0);

    auto algo = makeDijkstra(grid);
    algo.setConnectivity(finenav::Connectivity::N4);
    algo.setGoalTolerance(0.0);

    auto path = algo.findPath({2, 0, 0}, {2, 5, 0});
    ASSERT_TRUE(path.has_value());

    // The path should avoid x=2, y=1..3 (high-cost cells)
    for (const auto& c : *path) {
        EXPECT_FALSE(c.x == 2 && c.y >= 1 && c.y <= 3)
            << "Dijkstra took high-cost shortcut at (" << c.x << "," << c.y << ")";
    }
}

// ── BfsPathSearch integration tests ─────────────────────────────────────

class TestMapView : public finenav::BfsPathSearch::IMapView {
public:
    TestMapView(const finenav::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const finenav::Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getCost(const finenav::Position3D& p) const override {
        return grid_.getCostCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getStepSize() const override { return step_; }

private:
    const finenav::GridMap& grid_;
    double step_;
};

inline finenav::BfsPathSearch makePlanner()
{
    finenav::BfsPathSearch planner;
    bfs_planner::Params p;
    p.connectivity      = "4";
    p.goal_tolerance    = 0.6;
    p.max_iterations    = 1000000;
    p.max_planning_time = 5.0;
    p.movement_weight   = 1.0;
    planner.configure(p);
    return planner;
}

inline finenav::PlanningContext<finenav::BfsPathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::BfsPathSearch> ctx;
    ctx.robot_state.position.x() = sx;
    ctx.robot_state.position.y() = sy;
    ctx.goal_pose.position.x = gx;
    ctx.goal_pose.position.y = gy;
    return ctx;
}

TEST(BfsPathSearch, PlanSucceeds_OpenGrid)
{
    finenav::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();
    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(BfsPathSearch, PlanFails_GoalBlocked)
{
    finenav::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(BfsPathSearch, PlanFails_InvalidStepSize)
{
    finenav::GridMap grid(10, 10);
    TestMapView view(grid, 0.0);
    auto planner = makePlanner();
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(BfsPathSearch, OutputPoses_AreWorldCoordinates)
{
    finenav::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();
    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    for (const auto& pose : result->poses) {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}
