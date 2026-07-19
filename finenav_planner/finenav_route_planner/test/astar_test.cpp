// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file astar_test.cpp
 * @brief A* 算法单元测试 — 覆盖核心引擎正确性 + PlannerConcept 集成
 *
 * @details
 * ==========================================================================
 * 测试总览
 * ==========================================================================
 * 本文件使用 Google Test 验证 AStarAlgorithm 核心引擎和 AstarPathSearch
 * 包装器的正确性。测试按四个层次组织，从纯算法到 FineNav 框架集成逐层推进。
 *
 * 被测对象:
 *   - AStarAlgorithm (astar:: 命名空间, 无 ROS 依赖)
 *   - AstarPathSearch (finenav:: 命名空间, PlannerConcept 实现)
 *
 * 测试分层:
 *   -# 2D 核心算法 — 开放空间寻路、路径长度、连续性、对角 vs 正交、
 *      障碍规避 (lambda 回调, 不依赖 IMapView)
 *   -# 边界条件与搜索限制 — 起点=终点、起点/终点被障碍占据、封闭目标、
 *      max_iterations 截断
 *   -# 3D 搜索 — N26 连通性、N6 面邻居路径长度
 *   -# AstarPathSearch 集成 — TestMapView + PlanningContext + plan(),
 *      覆盖成功/失败/世界坐标输出
 *
 * ==========================================================================
 * 测试环境
 * ==========================================================================
 * 测试框架:     Google Test (GTest)
 * 依赖:         finenav_route_planner, common/grid_map.hpp (测试工具)
 * ROS 依赖:     无（核心算法层）, 仅 AstarPathSearch 集成层需要 ROS 2 头文件
 * 构建:         colcon build --packages-select finenav_route_planner
 *               自动作为 BUILD_TESTING 的一部分编译
 *
 * 运行:
 * @code
 *   cd ~/FineNav_ws
 *   colcon test --packages-select finenav_route_planner
 *   # 或直接运行可执行文件:
 *   ./build/finenav_route_planner/test/astar_test
 *   # 按名称过滤:
 *   ./build/finenav_route_planner/test/astar_test --gtest_filter="AStarAlgorithm2D.*"
 *   ./build/finenav_route_planner/test/astar_test --gtest_filter="AStarPathSearch.*"
 * @endcode
 *
 * ==========================================================================
 * 测试详细说明
 * ==========================================================================
 *
 * --- Group 1: AStarAlgorithm2D (核心算法, 2D) ---
 * FindsPath_OpenSpace             — 20×20 空网格, N4, (0,0)→(5,5) 可到达
 * PathLength_Orthogonal           — 直线 5 步, N4, 验证路径长度 = 6 cells
 * PathIsContinuous                — 验证相邻 cell 曼哈顿距离 = 1 (连续性)
 * Diagonal_AllowedReducesPathLength — N8 路径长度 < N4 路径长度 (对角线更短)
 * PathAvoidsObstacles             — x=3 处垂直墙, 验证路径不穿越障碍
 *
 * --- Group 2: 边界条件 ---
 * StartEqualsGoal                 — 起点=终点 → 返回单 cell 路径
 * StartBlocked_ReturnsNullopt     — 起点被占据 → nullopt
 * GoalBlocked_ReturnsNullopt      — 终点被占据 → nullopt
 * EnclosedGoal_ReturnsNullopt     — 终点被 8 邻域障碍包围, goal_tolerance=0 → nullopt
 * MaxIterations_TerminatesEarly   — 1000² 网格, max_iterations=10 → 不挂死
 *
 * --- Group 3: AStarAlgorithm3D ---
 * FindsPath_OpenVolume            — 全通 3D 空间, N26, (0,0,0)→(3,3,3)
 * PathLength_FaceNeighbours       — N6 (面邻居), (0,0,0)→(0,0,4) = 5 cells
 *
 * --- Group 4: AstarPathSearch 集成 ---
 * PlanSucceeds_OpenGrid           — 10×10 开放网格, goal_tolerance=0.6 → 成功
 * PlanFails_GoalBlocked            — 终点 cell 被占据 → nullopt
 * OutputPoses_AreWorldCoordinates  — 所有输出位姿的 x,y 在 [0,10) 范围内
 *
 * ==========================================================================
 * 设计说明
 * ==========================================================================
 *   - 核心算法测试使用 lambda 回调 (is_walkable, get_cost) 直接操作 GridMap,
 *     完全绕开 IMapView 抽象层, 保证测试的是纯算法逻辑
 *   - GridMap 按值捕获到 lambda 中 (makeGridLambdas 的 [g] 捕获),
 *     确保 lambda 拥有数据所有权, 临时 GridMap 对象也不会悬挂引用
 *   - AstarPathSearch 集成层使用 TestMapView 适配 GridMap,
 *     与生产环境的 OccGrid2DMap → IMapView 模式一致
 *   - 3D 测试复用 2D 算法 (Vec3 的 x/y 作为 grid 坐标, z 作为 layer),
 *     与代码中 DStarLiteAlgorithm 的多层支持对应
 *   - MaxIterations_TerminatesEarly 只验证不挂死, 不验证路径正确性
 *     (被截断后路径不完整是预期行为)
 *
 * @author Jinghui Wang (wangjinghui@sjtu.edu.cn)
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-07-19
 */

#include <gtest/gtest.h>
#include <cmath>
#include <unordered_set>

#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/astar_path_search.hpp"
#include "common/grid_map.hpp"    // test-only utility (in this directory)

// ── helpers ──────────────────────────────────────────────────────────────

// Build a simple walkable-everywhere lambda map for AStarAlgorithm.
inline auto makeOpenMap()
{
    return [](const astar::Vec3&) { return true; };
}

// Build a lambda map backed by GridMap.
// Takes GridMap by VALUE so lambdas own the data and are safe
// even when called with a temporary GridMap argument.
inline auto makeGridLambdas(astar::GridMap g, double /*res*/ = 1.0)
{
    auto is_walkable = [g](const astar::Vec3& c) -> bool {
        return g.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [g](const astar::Vec3& c) -> double {
        return g.getCostCell(c.x, c.y);
    };
    return std::make_pair(is_walkable, get_cost);
}

// ─────────────────────────────────────────────────────────────────────────
// Group 1: Core algorithm correctness (2-D)
// ─────────────────────────────────────────────────────────────────────────

TEST(AStarAlgorithm2D, FindsPath_OpenSpace)
{
    auto [is_walkable, get_cost] = makeGridLambdas(astar::GridMap(20, 20));
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (astar::Vec3{5, 5, 0}));
}

TEST(AStarAlgorithm2D, PathLength_Orthogonal)
{
    // Straight line of 5 steps → 6 cells including start
    auto [is_walkable, get_cost] = makeGridLambdas(astar::GridMap(20, 20));
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 0, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 6u);
}

TEST(AStarAlgorithm2D, PathIsContinuous)
{
    // Every consecutive pair of cells differs by exactly 1 in one axis.
    auto [is_walkable, get_cost] = makeGridLambdas(astar::GridMap(20, 20));
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({1, 1, 0}, {8, 8, 0});
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->size(), 2u);

    for (std::size_t i = 1; i < path->size(); ++i)
    {
        const auto& a = (*path)[i - 1];
        const auto& b = (*path)[i];
        int diff = std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
        EXPECT_EQ(diff, 1) << "Non-adjacent cells at index " << i;
    }
}

TEST(AStarAlgorithm2D, Diagonal_AllowedReducesPathLength)
{
    auto [iw, gc] = makeGridLambdas(astar::GridMap(20, 20));

    astar::AStarAlgorithm ortho(iw, gc);
    ortho.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);
    auto path_ortho = ortho.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_ortho.has_value());

    astar::AStarAlgorithm diag(iw, gc);
    diag.setConnectivity(astar::AStarAlgorithm::Connectivity::N8);
    auto path_diag = diag.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_diag.has_value());

    EXPECT_LT(path_diag->size(), path_ortho->size());
}

TEST(AStarAlgorithm2D, PathAvoidsObstacles)
{
    astar::GridMap grid(10, 10);
    // Vertical wall at x=3, y=0..8
    for (int y = 0; y <= 8; ++y)
        grid.setObstacle(3, y, true);

    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 5, 0}, {6, 5, 0});
    ASSERT_TRUE(path.has_value());

    // No cell in the path may be on the wall
    for (const auto& c : *path)
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
}

// ─────────────────────────────────────────────────────────────────────────
// Group 2: Edge cases and search limits
// ─────────────────────────────────────────────────────────────────────────

TEST(AStarAlgorithm2D, StartEqualsGoal)
{
    auto [iw, gc] = makeGridLambdas(astar::GridMap(10, 10));
    astar::AStarAlgorithm algo(iw, gc);
    auto path = algo.findPath({3, 3, 0}, {3, 3, 0});
    // A single-cell path is valid
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (astar::Vec3{3, 3, 0}));
}

TEST(AStarAlgorithm2D, StartBlocked_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(0, 0, true);
    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(AStarAlgorithm2D, GoalBlocked_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(AStarAlgorithm2D, EnclosedGoal_ReturnsNullopt)
{
    astar::GridMap grid(10, 10);
    // Surround goal (5,5) with a solid ring
    for (int d = -1; d <= 1; ++d) {
        grid.setObstacle(4, 5 + d, true);
        grid.setObstacle(6, 5 + d, true);
    }
    grid.setObstacle(5, 4, true);
    grid.setObstacle(5, 6, true);

    auto [iw, gc] = makeGridLambdas(grid);
    astar::AStarAlgorithm algo(iw, gc);
    algo.setGoalTolerance(0.0);  // exact match required
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(AStarAlgorithm2D, MaxIterations_TerminatesEarly)
{
    auto [iw, gc] = makeGridLambdas(astar::GridMap(1000, 1000));
    astar::AStarAlgorithm algo(iw, gc);
    algo.setMaxIterations(10);
    algo.setGoalTolerance(0.0);
    // Very distant goal — should be cut off
    auto path = algo.findPath({0, 0, 0}, {999, 999, 0});
    // Either no path (cut off) or a sub-optimal partial
    // The important thing is it does NOT run forever
    (void)path;
    SUCCEED();
}

// ─────────────────────────────────────────────────────────────────────────
// Group 3: 3-D search
// ─────────────────────────────────────────────────────────────────────────

TEST(AStarAlgorithm3D, FindsPath_OpenVolume)
{
    // All-walkable 3D space via lambda
    auto is_walkable = [](const astar::Vec3&) { return true; };
    auto get_cost    = [](const astar::Vec3&) { return 0.0; };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N26);

    auto path = algo.findPath({0, 0, 0}, {3, 3, 3});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(),  (astar::Vec3{3, 3, 3}));
}

TEST(AStarAlgorithm3D, PathLength_FaceNeighbours)
{
    // With face-only neighbours, cost from (0,0,0) to (0,0,4) = 5 cells
    auto is_walkable = [](const astar::Vec3&) { return true; };
    auto get_cost    = [](const astar::Vec3&) { return 0.0; };

    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(astar::AStarAlgorithm::Connectivity::N6);

    auto path = algo.findPath({0, 0, 0}, {0, 0, 4});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 5u);
}

// ─────────────────────────────────────────────────────────────────────────
// Group 4: AstarPathSearch integration (IMapView + planner)
// ─────────────────────────────────────────────────────────────────────────

// Minimal IMapView implementation backed by GridMap
class TestMapView : public finenav::AstarPathSearch::IMapView {
public:
    TestMapView(const astar::GridMap& g, double step_size)
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
    const astar::GridMap& grid_;
    double step_;
};

// Helper: build a default-configured AstarPathSearch
inline finenav::AstarPathSearch makePlanner()
{
    finenav::AstarPathSearch planner;
    astar_planner::Params p;
    p.connectivity    = "4";
    p.heuristic       = "Euclidean";
    p.goal_tolerance  = 0.6;   // metres
    p.max_iterations  = 1000000;
    p.max_planning_time = 5.0;
    planner.configure(p);
    return planner;
}

// Helper: build a PlanningContext from cell centres
inline finenav::PlanningContext<finenav::AstarPathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::AstarPathSearch> ctx;
    ctx.robot_state.position.x() = sx;
    ctx.robot_state.position.y() = sy;
    ctx.goal_pose.position.x  = gx;
    ctx.goal_pose.position.y  = gy;
    return ctx;
}

TEST(AstarPathSearch, PlanSucceeds_OpenGrid)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(AstarPathSearch, PlanFails_GoalBlocked)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    EXPECT_FALSE(result.has_value());
}

TEST(AstarPathSearch, OutputPoses_AreWorldCoordinates)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // All poses should be within the grid bounds (cell centres at x.5)
    for (const auto& pose : result->poses)
    {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}





