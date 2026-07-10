// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// DStarLitePathSearch / D* Lite test suite.
//
// Tests are organised in two layers matching the existing
// astar_test / bfs_test pattern:
//
//   Layer 1 — Core DStarLiteAlgorithm tests (lambda callbacks, no IMapView).
//   Layer 2 — DStarLitePathSearch integration tests (TestMapView + PlanningContext).

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "finenav_route_planner/dstarlite_algorithm.hpp"
#include "finenav_route_planner/dstarlite_path_search.hpp"
#include "grid_map.hpp"

// ═══════════════════════════════════════════════════════════════════
// Shared helpers
// ═══════════════════════════════════════════════════════════════════

// Build a DStarLiteAlgorithm over a GridMap using lambda callbacks.
inline astar::DStarLiteAlgorithm makeEngine(
    const astar::GridMap& g,
    astar::Connectivity  conn = astar::Connectivity::N8,
    astar::Heuristic     heur = astar::Heuristic::Diagonal)
{
    auto is_walkable = [g](const astar::Vec3& c) {
        return g.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [g](const astar::Vec3& c) {
        return g.getCostCell(c.x, c.y);
    };
    return astar::DStarLiteAlgorithm(g.getHeight(), g.getWidth(),
                                 is_walkable, get_cost, conn, heur);
}

// ═══════════════════════════════════════════════════════════════════
// Group 1: Core DStarLiteAlgorithm tests (grid-space, no IMapView)
// ═══════════════════════════════════════════════════════════════════

// ── Basic paths ──────────────────────────────────────────────────

TEST(DStarLiteAlgorithm, FindsPath_OpenSpace)
{
    auto engine = makeEngine(astar::GridMap(20, 20));
    engine.initialize({0, 0, 0}, {5, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path.back(),  (astar::Vec3{5, 5, 0}));
}

TEST(DStarLiteAlgorithm, StartEqualsGoal)
{
    auto engine = makeEngine(astar::GridMap(10, 10));
    engine.initialize({3, 3, 0}, {3, 3, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), (astar::Vec3{3, 3, 0}));
}

TEST(DStarLiteAlgorithm, Diagonal_ShorterPath)
{
    auto ortho = makeEngine(astar::GridMap(20, 20), astar::Connectivity::N4);
    ortho.initialize({0, 0, 0}, {5, 5, 0});
    ortho.computeShortestPath();
    auto path_ortho = ortho.extractPath();

    auto diag = makeEngine(astar::GridMap(20, 20), astar::Connectivity::N8);
    diag.initialize({0, 0, 0}, {5, 5, 0});
    diag.computeShortestPath();
    auto path_diag = diag.extractPath();

    ASSERT_FALSE(path_ortho.empty());
    ASSERT_FALSE(path_diag.empty());
    EXPECT_LT(path_diag.size(), path_ortho.size());
}

TEST(DStarLiteAlgorithm, PathIsContinuous_N4)
{
    auto engine = makeEngine(astar::GridMap(10, 10), astar::Connectivity::N4);
    engine.initialize({1, 1, 0}, {5, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_GE(path.size(), 2u);
    for (size_t i = 1; i < path.size(); ++i) {
        int diff = std::abs(path[i].x - path[i-1].x)
                 + std::abs(path[i].y - path[i-1].y)
                 + std::abs(path[i].z - path[i-1].z);
        EXPECT_EQ(diff, 1) << "Non-adjacent cells at index " << i;
    }
}

// ── Obstacle handling ────────────────────────────────────────────

TEST(DStarLiteAlgorithm, GoalBlocked_ReturnsEmpty)
{
    astar::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    auto engine = makeEngine(grid);
    engine.initialize({0, 0, 0}, {5, 5, 0});
    engine.computeShortestPath();
    EXPECT_TRUE(engine.extractPath().empty());
}

TEST(DStarLiteAlgorithm, PathAvoidsObstacles)
{
    astar::GridMap grid(10, 10);
    // Build a vertical wall at x=3
    for (int y = 0; y <= 8; ++y) grid.setObstacle(3, y, true);

    auto engine = makeEngine(grid, astar::Connectivity::N4);
    engine.initialize({0, 5, 0}, {6, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    for (const auto& c : path) {
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
    }
}

// ── Incremental replanning ───────────────────────────────────────

TEST(DStarLiteAlgorithm, Replan_AdaptsToNewObstacle)
{
    auto engine = makeEngine(astar::GridMap(10, 10));
    engine.initialize({0, 0, 0}, {9, 9, 0});
    engine.computeShortestPath();
    auto path1 = engine.extractPath();
    ASSERT_FALSE(path1.empty());

    // Block the path with a wall
    for (int y = 0; y < 7; ++y)
        engine.setObstacle({5, y, 0});

    engine.replan({0, 0, 0});
    auto path2 = engine.extractPath();
    ASSERT_FALSE(path2.empty());

    // New path must not pass through blocked cells
    for (const auto& c : path2) {
        EXPECT_FALSE(c.x == 5 && c.y < 7)
            << "Replanned path passes through obstacle at ("
            << c.x << "," << c.y << ")";
    }
}

TEST(DStarLiteAlgorithm, RemoveObstacle_OpensNewPath)
{
    // Full vertical wall at x=5
    astar::GridMap grid(10, 10);
    for (int y = 0; y <= 9; ++y) grid.setObstacle(5, y, true);
    auto engine = makeEngine(grid);

    engine.initialize({0, 5, 0}, {9, 5, 0});
    engine.computeShortestPath();
    auto blocked_path = engine.extractPath();
    EXPECT_TRUE(blocked_path.empty()) << "Should be blocked by wall";

    // Remove one cell to open a gap
    engine.removeObstacle({5, 5, 0});
    engine.replan({0, 5, 0});
    auto open_path = engine.extractPath();

    ASSERT_FALSE(open_path.empty());
    bool passes_gap = false;
    for (const auto& c : open_path) {
        if (c.x == 5 && c.y == 5) { passes_gap = true; break; }
    }
    EXPECT_TRUE(passes_gap);
}

TEST(DStarLiteAlgorithm, SyncMap_DetectsChanges)
{
    // Start with an all-free map, then add an obstacle externally.
    astar::GridMap grid(10, 10);

    auto is_walkable = [&grid](const astar::Vec3& c) {
        return grid.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [&grid](const astar::Vec3& c) {
        return grid.getCostCell(c.x, c.y);
    };

    astar::DStarLiteAlgorithm engine(grid.getHeight(), grid.getWidth(),
                                 is_walkable, get_cost);

    engine.initialize({0, 0, 0}, {9, 9, 0});
    engine.computeShortestPath();
    auto path1 = engine.extractPath();
    ASSERT_FALSE(path1.empty());

    // Modify the grid externally — engine doesn't know yet.
    grid.setObstacle(5, 5, true);

    // syncMap discovers the change and triggers vertex updates.
    engine.syncMap();
    engine.replan({0, 0, 0});
    auto path2 = engine.extractPath();
    ASSERT_FALSE(path2.empty());

    // The new path must not go through the newly-blocked cell.
    for (const auto& c : path2) {
        EXPECT_FALSE(c.x == 5 && c.y == 5)
            << "Path passes through newly-blocked cell";
    }
}

// ── Limits ──────────────────────────────────────────────────────

TEST(DStarLiteAlgorithm, MaxExpansions_TerminatesEarly)
{
    auto engine = makeEngine(astar::GridMap(1000, 1000));
    engine.setMaxExpansions(50);
    engine.setGoalTolerance(0);
    engine.initialize({0, 0, 0}, {999, 999, 0});
    engine.computeShortestPath();
    // Should not hang — test that it terminates.
    SUCCEED();
}

// ── Goal tolerance ──────────────────────────────────────────────

TEST(DStarLiteAlgorithm, GoalTolerance_NearGoalSuccess)
{
    astar::GridMap grid(10, 10);
    // Block goal (5,5) and its immediate Manhattan neighbours
    grid.setObstacle(5, 5, true);
    grid.setObstacle(4, 5, true);
    grid.setObstacle(5, 4, true);
    grid.setObstacle(6, 5, true);
    grid.setObstacle(5, 6, true);

    auto engine = makeEngine(grid);
    engine.setGoalTolerance(2);
    engine.initialize({0, 0, 0}, {5, 5, 0});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    // The effective goal should be within 2 cells of (5,5)
    int dist = std::abs(path.back().x - 5) + std::abs(path.back().y - 5);
    EXPECT_LE(dist, 2);
}

// ── 3-D ─────────────────────────────────────────────────────────

TEST(DStarLiteAlgorithm, ThreeD_FindsPath_OpenVolume)
{
    auto walk = [](const astar::Vec3&) { return true; };
    auto cost = [](const astar::Vec3&) { return 0.0; };

    astar::DStarLiteAlgorithm engine(5, 5, walk, cost,
                                 astar::Connectivity::N26,
                                 astar::Heuristic::Diagonal,
                                 5);  // 5 layers
    engine.initialize({0, 0, 0}, {3, 3, 3});
    engine.computeShortestPath();
    auto path = engine.extractPath();

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), (astar::Vec3{0, 0, 0}));
    EXPECT_EQ(path.back(),  (astar::Vec3{3, 3, 3}));
}

// ═══════════════════════════════════════════════════════════════════
// Group 2: DStarLitePathSearch integration tests (IMapView + planner)
// ═══════════════════════════════════════════════════════════════════

// TestMapView: wraps GridMap into DStarLitePathSearch::IMapView.
// Must provide isWalkable, getCost, getStepSize (from astar::IMapView)
// PLUS rows, cols, layers (from DStarLitePathSearch::IMapView).
class TestMapView : public finenav::DStarLitePathSearch::IMapView {
public:
    TestMapView(const astar::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const finenav::core::Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getCost(const finenav::core::Position3D& p) const override {
        return grid_.getCostCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }
    double getStepSize() const override { return step_; }

    int rows()   const override { return grid_.getHeight(); }
    int cols()   const override { return grid_.getWidth(); }
    int layers() const override { return 1; }

private:
    const astar::GridMap& grid_;
    double step_;
};

inline void configurePlanner(finenav::DStarLitePathSearch& planner)
{
    dstarlite_planner::Params p;
    p.connectivity      = "8";
    p.heuristic         = "Diagonal";
    p.goal_tolerance    = 0.6;
    p.max_expansions    = 1000000;
    p.max_path_steps    = 10000;
    p.max_planning_time = 5.0;
    planner.configure(p);
}

inline finenav::PlanningContext<finenav::DStarLitePathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::DStarLitePathSearch> ctx;
    ctx.robot_state.position.x() = sx;
    ctx.robot_state.position.y() = sy;
    ctx.goal_pose.position.x    = gx;
    ctx.goal_pose.position.y    = gy;
    return ctx;
}

TEST(DStarLitePathSearch, PlanSucceeds_OpenGrid)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(DStarLitePathSearch, PlanFails_GoalBlocked)
{
    astar::GridMap grid(10, 10);
    // Block the goal and all neighbours so goal tolerance can't rescue.
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            grid.setObstacle(5 + dx, 5 + dy, true);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(DStarLitePathSearch, PlanFails_InvalidStepSize)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 0.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view).has_value());
}

TEST(DStarLitePathSearch, OutputPoses_AreWorldCoordinates)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);
    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    for (const auto& pose : result->poses) {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}

TEST(DStarLitePathSearch, ReplanAfterObstacleChange)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);

    // First plan: open grid.
    auto result1 = planner.plan(makeCtx(0.5, 0.5, 9.5, 9.5), view);
    ASSERT_TRUE(result1.has_value());
    ASSERT_GE(result1->poses.size(), 1u);

    // Add obstacles on the original path.
    grid.setObstacle(5, 5, true);
    grid.setObstacle(5, 4, true);
    grid.setObstacle(5, 6, true);

    // Replan — same planner instance, same goal, updated map.
    auto result2 = planner.plan(makeCtx(0.5, 0.5, 9.5, 9.5), view);
    ASSERT_TRUE(result2.has_value());

    // The new path must avoid the obstacles.
    for (const auto& pose : result2->poses) {
        int cx = static_cast<int>(std::floor(pose.position.x));
        int cy = static_cast<int>(std::floor(pose.position.y));
        EXPECT_FALSE(cx == 5 && cy == 5)
            << "Replanned path passes through obstacle at (" << cx << "," << cy << ")";
        EXPECT_FALSE(cx == 5 && cy == 4);
        EXPECT_FALSE(cx == 5 && cy == 6);
    }
}

TEST(DStarLitePathSearch, Reset_ClearsEngineState)
{
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    finenav::DStarLitePathSearch planner;
    configurePlanner(planner);

    // Initial plan.
    auto result1 = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result1.has_value());

    // Reset — should clear the engine.
    planner.reset();

    // After reset, a new plan with a different goal should work.
    auto result2 = planner.plan(makeCtx(0.5, 0.5, 8.5, 2.5), view);
    ASSERT_TRUE(result2.has_value());
}
