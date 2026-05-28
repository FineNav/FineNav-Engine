// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>

#include "bfs_algorithm.hpp"
#include "bfs_path_search.hpp"
#include "grid_map.hpp"

inline auto makeGridWalkable(bfs::GridMap g)
{
    return [g](const bfs::Vec3& c) -> bool {
        return g.isWalkableCell(c.x, c.y);
    };
}

TEST(BfsAlgorithm2D, FindsPath_OpenSpace)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(20, 20)));
    algo.setConnectivity(bfs::BfsAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (bfs::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(), (bfs::Vec3{5, 5, 0}));
}

TEST(BfsAlgorithm2D, PathLength_Orthogonal)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(20, 20)));
    algo.setConnectivity(bfs::BfsAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 0, 0}, {5, 0, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 6u);
}

TEST(BfsAlgorithm2D, Diagonal_AllowedReducesPathLength)
{
    auto walkable = makeGridWalkable(bfs::GridMap(20, 20));

    bfs::BfsAlgorithm ortho(walkable);
    ortho.setConnectivity(bfs::BfsAlgorithm::Connectivity::N4);
    auto path_ortho = ortho.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_ortho.has_value());

    bfs::BfsAlgorithm diag(walkable);
    diag.setConnectivity(bfs::BfsAlgorithm::Connectivity::N8);
    auto path_diag = diag.findPath({0, 0, 0}, {5, 5, 0});
    ASSERT_TRUE(path_diag.has_value());

    EXPECT_LT(path_diag->size(), path_ortho->size());
}

TEST(BfsAlgorithm2D, PathAvoidsObstacles)
{
    bfs::GridMap grid(10, 10);
    for (int y = 0; y <= 8; ++y) {
        grid.setObstacle(3, y, true);
    }

    bfs::BfsAlgorithm algo(makeGridWalkable(grid));
    algo.setConnectivity(bfs::BfsAlgorithm::Connectivity::N4);

    auto path = algo.findPath({0, 5, 0}, {6, 5, 0});
    ASSERT_TRUE(path.has_value());

    for (const auto& c : *path) {
        EXPECT_FALSE(c.x == 3 && c.y <= 8)
            << "Path passes through obstacle at (" << c.x << "," << c.y << ")";
    }
}

TEST(BfsAlgorithm2D, StartEqualsGoal)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(10, 10)));
    auto path = algo.findPath({3, 3, 0}, {3, 3, 0});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (bfs::Vec3{3, 3, 0}));
}

TEST(BfsAlgorithm2D, GoalBlocked_ReturnsNullopt)
{
    bfs::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);

    bfs::BfsAlgorithm algo(makeGridWalkable(grid));
    EXPECT_FALSE(algo.findPath({0, 0, 0}, {5, 5, 0}).has_value());
}

TEST(BfsAlgorithm2D, MaxIterations_TerminatesEarly)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(1000, 1000)));
    algo.setMaxIterations(10);
    algo.setGoalTolerance(0.0);
    (void)algo.findPath({0, 0, 0}, {999, 999, 0});
    SUCCEED();
}

TEST(BfsAlgorithm2D, MaxIterationsZero_ReturnsNulloptForDifferentGoal)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(10, 10)));
    algo.setMaxIterations(0);
    algo.setGoalTolerance(0.0);

    EXPECT_FALSE(algo.findPath({0, 0, 0}, {1, 0, 0}).has_value());
}

TEST(BfsAlgorithm2D, FindPathToAny_ReturnsStartWhenStartIsGoal)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(10, 10)));

    auto path = algo.findPathToAny({3, 3, 0}, [](const bfs::Vec3& c) {
        return c == bfs::Vec3{3, 3, 0};
    });

    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (bfs::Vec3{3, 3, 0}));
}

TEST(BfsAlgorithm2D, FindPathToAny_StopsAtFirstReachableGoal)
{
    bfs::BfsAlgorithm algo(makeGridWalkable(bfs::GridMap(20, 20)));
    algo.setConnectivity(bfs::BfsAlgorithm::Connectivity::N8);

    auto path = algo.findPathToAny({0, 0, 0}, [](const bfs::Vec3& c) {
        return c == bfs::Vec3{3, 0, 0} || c == bfs::Vec3{8, 0, 0};
    });

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->back(), (bfs::Vec3{3, 0, 0}));
    EXPECT_EQ(path->size(), 4u);
}

TEST(BfsAlgorithm3D, FindsPath_OpenVolume)
{
    auto is_walkable = [](const bfs::Vec3&) { return true; };

    bfs::BfsAlgorithm algo(is_walkable);
    algo.setConnectivity(bfs::BfsAlgorithm::Connectivity::N26);

    auto path = algo.findPath({0, 0, 0}, {3, 3, 3});
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (bfs::Vec3{0, 0, 0}));
    EXPECT_EQ(path->back(), (bfs::Vec3{3, 3, 3}));
}

class TestMapView : public finenav::BfsPathSearch::IMapView {
public:
    TestMapView(const bfs::GridMap& g, double step_size)
        : grid_(g), step_(step_size) {}

    bool isWalkable(const finenav::Position3D& p) const override {
        return grid_.isWalkableCell(
            static_cast<int>(std::floor(p.x() / step_)),
            static_cast<int>(std::floor(p.y() / step_)));
    }

    double getStepSize() const override { return step_; }

private:
    const bfs::GridMap& grid_;
    double step_;
};

inline finenav::BfsPathSearch makePlanner()
{
    finenav::BfsPathSearch planner;
    bfs_planner::Params p;
    p.connectivity = "4";
    p.goal_tolerance = 0.6;
    p.max_iterations = 1000000;
    p.max_planning_time = 5.0;
    planner.configure(p);
    return planner;
}

inline finenav::PlanningContext<finenav::BfsPathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::BfsPathSearch> ctx;
    ctx.robot_state.pose.position.x = sx;
    ctx.robot_state.pose.position.y = sy;
    ctx.goal_pose.position.x = gx;
    ctx.goal_pose.position.y = gy;
    return ctx;
}

TEST(BfsPathSearch, PlanSucceeds_OpenGrid)
{
    bfs::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_GE(result->poses.size(), 1u);
}

TEST(BfsPathSearch, PlanFails_GoalBlocked)
{
    bfs::GridMap grid(10, 10);
    grid.setObstacle(5, 5, true);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    EXPECT_FALSE(result.has_value());
}

TEST(BfsPathSearch, PlanFails_InvalidStepSize)
{
    bfs::GridMap grid(10, 10);
    TestMapView view(grid, 0.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 5.5, 5.5), view);
    EXPECT_FALSE(result.has_value());
}

TEST(BfsPathSearch, OutputPoses_AreWorldCoordinates)
{
    bfs::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);
    auto planner = makePlanner();

    auto result = planner.plan(makeCtx(0.5, 0.5, 3.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    for (const auto& pose : result->poses)
    {
        EXPECT_GE(pose.position.x, 0.0);
        EXPECT_LT(pose.position.x, 10.0);
        EXPECT_GE(pose.position.y, 0.0);
        EXPECT_LT(pose.position.y, 10.0);
    }
}
