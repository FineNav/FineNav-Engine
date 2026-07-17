// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// DetourPlanner test suite.
//
// Covers:
//   - Output contract: the returned path always starts at the robot pose
//     (re-anchoring), including when the robot was displaced off the
//     reference (hijack) or sits mid-path
//   - Lethal blockage (!isWalkable on the path) → detour around, rejoin
//   - Soft blockage (cost >= blockage_threshold, still walkable) → detour
//   - Obstacle covering a long stretch: unreachable reconnect candidates
//     are skipped automatically, path rejoins beyond the obstacle
//   - Fallback pass-through: empty reference, no reachable candidate

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "detour_planner.hpp"
#include "grid_map.hpp"

namespace {

// ── Test map view (same adaptor pattern as route_test.cpp) ───────────────
class TestMapView : public astar::IMapView {
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

// ── helpers ──────────────────────────────────────────────────────────────

finenav::DetourPlanner makePlanner()
{
    finenav::DetourPlanner planner;
    detour_planner::Params p;
    p.scan_distance_m      = 10.0;
    p.blockage_threshold   = 0.8;
    p.reconnect_offset_m   = 1.0;
    p.obstacle_cost_weight = 1.0;
    p.max_planning_time    = 1.0;
    planner.configure(p);
    return planner;
}

// Straight reference path along y = `y`, x from x0 to x1 at `spacing`.
std::vector<geometry_msgs::msg::Pose> makeRefPath(
    double x0, double x1, double y, double spacing = 0.5)
{
    std::vector<geometry_msgs::msg::Pose> poses;
    for (double x = x0; x <= x1 + 1e-9; x += spacing) {
        geometry_msgs::msg::Pose p;
        p.position.x    = x;
        p.position.y    = y;
        p.orientation.w = 1.0;
        poses.push_back(p);
    }
    return poses;
}

finenav::PlanningContext<finenav::DetourPlanner> makeCtx(
    double rx, double ry, std::vector<geometry_msgs::msg::Pose> ref)
{
    finenav::PlanningContext<finenav::DetourPlanner> ctx;
    ctx.robot_state.pose.position.x    = rx;
    ctx.robot_state.pose.position.y    = ry;
    ctx.robot_state.pose.orientation.w = 1.0;
    ctx.ref_traj.poses = std::move(ref);
    return ctx;
}

void expectSamePath(const std::vector<geometry_msgs::msg::Pose>& a,
                    const std::vector<geometry_msgs::msg::Pose>& b)
{
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_NEAR(a[i].position.x, b[i].position.x, 1e-9);
        EXPECT_NEAR(a[i].position.y, b[i].position.y, 1e-9);
    }
}

}  // namespace

// =========================================================================

TEST(DetourPlanner, EmptyReference_ReturnsEmptyOutput)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, {}), view);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->poses.empty());
}

TEST(DetourPlanner, ClearPath_RobotAtStart_OutputIdentical)
{
    // Robot standing exactly on the first reference waypoint: re-anchoring
    // degenerates to the identity — output equals the reference.
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(0.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    expectSamePath(result->poses, ref);
}

TEST(DetourPlanner, ClearPath_RobotMidPath_DropsWaypointsBehind)
{
    // Robot on the path but halfway along it: output starts at the robot
    // and the reference prefix behind the robot is dropped.
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(8.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    EXPECT_NEAR(result->poses.front().position.x, 8.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);

    for (const auto& p : result->poses)
        EXPECT_GE(p.position.x, 8.5 - 1e-6)
            << "Kept waypoint behind robot: x=" << p.position.x;
}

TEST(DetourPlanner, HijackedRobot_ReanchorsFromRobot)
{
    // Robot displaced off the reference (manual override), with a wall
    // between it and the path.  The output must start at the robot and
    // reconnect to the reference around the wall — this is the case that
    // makes pass-through unacceptable.
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall row y=2, x=0..4: blocks the straight line from robot to path.
    for (int x = 0; x <= 4; ++x) grid.setObstacle(x, 2, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 4.5, ref), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // Starts at the (displaced) robot pose.
    EXPECT_NEAR(result->poses.front().position.x, 2.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 4.5, 1e-6);

    // Never crosses the wall.
    for (const auto& p : result->poses)
        EXPECT_FALSE(p.position.x < 5.0 && p.position.y >= 2.0 && p.position.y < 3.0)
            << "Crossed wall at (" << p.position.x << ", " << p.position.y << ")";

    // Rejoins the reference and reaches its end.
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);
}

TEST(DetourPlanner, LethalBlockage_DetoursAroundAndRejoins)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall at x=8, rows y=0..2 — blocks the path, forces a detour over y=3.
    for (int y = 0; y <= 2; ++y) grid.setObstacle(8, y, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // Output starts at the robot — no reference prefix doubling back.
    EXPECT_NEAR(result->poses.front().position.x, 2.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // Ends at the reference end.
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);

    // Never enters the wall column below the gap.
    for (const auto& p : result->poses)
        EXPECT_FALSE(p.position.x > 8.0 && p.position.x < 9.0 && p.position.y < 3.0)
            << "Entered wall at (" << p.position.x << ", " << p.position.y << ")";

    // The detour climbs over the wall (some point at y >= 3).
    bool climbed = false;
    for (const auto& p : result->poses)
        if (p.position.y >= 3.0) climbed = true;
    EXPECT_TRUE(climbed);
}

TEST(DetourPlanner, SoftBlockage_TriggersOnCostThreshold)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // High cost (still walkable) on the path cell at x=8.
    grid.setCost(8, 0, 0.95);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());

    // A detour happened: the output bulges away from the straight line
    // (the cost-weighted search sidesteps the expensive cell).
    bool bulged = false;
    for (const auto& p : result->poses)
        if (p.position.y > 1.0) bulged = true;
    EXPECT_TRUE(bulged);

    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);
}

TEST(DetourPlanner, LongObstacle_SkipsUnreachableCandidates_RejoinsBeyond)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall covering x=6..10, rows y=0..2: candidates within
    // reconnect_offset_m of the blockage are unreachable (inside the wall)
    // and must be skipped automatically.
    for (int x = 6; x <= 10; ++x)
        for (int y = 0; y <= 2; ++y)
            grid.setObstacle(x, y, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());

    // While crossing the wall span, the path must stay above it.
    for (const auto& p : result->poses)
        EXPECT_FALSE(p.position.x > 6.0 && p.position.x < 11.0 && p.position.y < 3.0)
            << "Entered wall at (" << p.position.x << ", " << p.position.y << ")";

    // Rejoins the reference beyond the wall and reaches its end.
    bool rejoined = false;
    for (const auto& p : result->poses)
        if (std::abs(p.position.y - 0.5) < 1e-6 && p.position.x >= 11.0) rejoined = true;
    EXPECT_TRUE(rejoined);
    EXPECT_NEAR(result->poses.back().position.x, 15.5, 1e-6);
}

TEST(DetourPlanner, FullyBlocked_PassesThrough)
{
    auto planner = makePlanner();
    astar::GridMap grid(20, 10);
    // Wall across the full grid height at x=8 — no reconnect candidate is
    // reachable; the planner must warn and pass the reference through.
    for (int y = 0; y < 10; ++y) grid.setObstacle(8, y, true);
    TestMapView view(grid, 1.0);

    const auto ref = makeRefPath(0.5, 15.5, 0.5);
    auto result = planner.plan(makeCtx(2.5, 0.5, ref), view);
    ASSERT_TRUE(result.has_value());
    expectSamePath(result->poses, ref);
}
