// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// RoutePathSearch (topological route planning) test suite.
//
// Covers:
//   - GeoJSON graph loading via configure() (valid / missing / malformed)
//   - Dijkstra routing on the topological graph (reachable / disconnected /
//     fixed-cost-override route selection / start==goal snapping)
//   - Obstacle-aware pose-to-node snapping (findNearestNodeIndex), including
//     skipping a geometrically-closer but unreachable node
//   - Output shape: world-frame poses, densification, exact endpoints

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "route_path_search.hpp"
#include "grid_map.hpp"

namespace {

namespace fs = std::filesystem;

// ── RAII temp GeoJSON file ───────────────────────────────────────────────
// RoutePathSearch loads its graph from a file path (graph_file param), so
// each fixture is materialised on disk for the duration of the test.
class TempFile {
public:
    explicit TempFile(const std::string& content)
        : path_((fs::temp_directory_path() /
                 ("route_planner_test_" + std::to_string(counter_++) + ".geojson")).string())
    {
        std::ofstream f(path_);
        f << content;
    }
    ~TempFile() { std::error_code ec; fs::remove(path_, ec); }

    const std::string& path() const { return path_; }

private:
    std::string path_;
    static inline int counter_ = 0;
};

// ── Test map view (same adaptor pattern as bfs_test.cpp) ────────────────
// Wraps the dependency-free astar::GridMap as an astar::IMapView so the
// obstacle-aware snapping inside RoutePathSearch can be exercised.
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

finenav::RoutePathSearch makePlanner(const std::string& graph_file, double path_density = 0.05)
{
    finenav::RoutePathSearch planner;
    route_planner::Params p;
    p.graph_file   = graph_file;
    p.path_density = path_density;
    planner.configure(p);
    return planner;
}

finenav::PlanningContext<finenav::RoutePathSearch>
makeCtx(double sx, double sy, double gx, double gy)
{
    finenav::PlanningContext<finenav::RoutePathSearch> ctx;
    ctx.robot_state.pose.position.x    = sx;
    ctx.robot_state.pose.position.y    = sy;
    ctx.robot_state.pose.orientation.w = 1.0;
    ctx.goal_pose.position.x    = gx;
    ctx.goal_pose.position.y    = gy;
    ctx.goal_pose.orientation.w = 1.0;
    return ctx;
}

double dist(double x0, double y0, double x1, double y1)
{
    return std::hypot(x1 - x0, y1 - y0);
}

double pathLength(const std::vector<geometry_msgs::msg::Pose>& poses)
{
    double len = 0.0;
    for (size_t i = 1; i < poses.size(); ++i) {
        len += dist(poses[i - 1].position.x, poses[i - 1].position.y,
                    poses[i].position.x,     poses[i].position.y);
    }
    return len;
}

// Surround cell (cx, cy) with obstacles on all 8 neighbours, isolating it
// from the rest of an otherwise-open N8-connected grid.
void enclose(astar::GridMap& grid, int cx, int cy)
{
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            if (dx != 0 || dy != 0)
                grid.setObstacle(cx + dx, cy + dy, true);
}

// ── GeoJSON fixtures ─────────────────────────────────────────────────────

// Linear chain  A(0,0) -- B(5,0) -- C(10,0), bidirectional edges.
const char* kLinearGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5,0]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":2,"frame":"C"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[5,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[5,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[5,0],[10,0]]},"properties":{"id":102,"startid":1,"endid":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[5,0]]},"properties":{"id":103,"startid":2,"endid":1}}
  ]
}
)json";

// Two disjoint chains: {0,1} near the origin, {2,3} far away — no edge joins them.
const char* kDisconnectedGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5,0]},"properties":{"id":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[20,0]},"properties":{"id":2}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[25,0]},"properties":{"id":3}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[5,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[5,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[20,0],[25,0]]},"properties":{"id":102,"startid":2,"endid":3}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[25,0],[20,0]]},"properties":{"id":103,"startid":3,"endid":2}}
  ]
}
)json";

// 0 --(fixed cost 100, overridable=false)--> 1   [direct, geometrically length 10]
// 0 -> 2 -> 1 with default (geometric-length) costs: 8 + sqrt(10^2+8^2) ~= 20.8
// The cheaper route must go through node 2 despite being geometrically longer.
const char* kCostOverrideGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":1}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,8]},"properties":{"id":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[10,0]]},"properties":{"id":100,"startid":0,"endid":1,"cost":100.0,"overridable":false}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[0,8]]},"properties":{"id":101,"startid":0,"endid":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,8],[10,0]]},"properties":{"id":102,"startid":2,"endid":1}}
  ]
}
)json";

// Two nodes: P sits in a pocket that obstacles will isolate, Q sits in the open.
const char* kSnapGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[2.5,2.5]},"properties":{"id":0,"frame":"P_enclosed"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[6.5,0.5]},"properties":{"id":1,"frame":"Q_open"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[2.5,2.5],[6.5,0.5]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[6.5,0.5],[2.5,2.5]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

// Single isolated node — used to test the "no node reachable at all" failure path.
const char* kIsolatedNodeGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[5.5,5.5]},"properties":{"id":0}}
  ]
}
)json";

}  // namespace

// =========================================================================
// configure() / graph loading
// =========================================================================

TEST(RoutePathSearch, ConfigureFails_GracefullyOnMissingFile)
{
    auto planner = makePlanner("/nonexistent/path/to/graph.geojson");
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, ConfigureFails_GracefullyOnMalformedJson)
{
    TempFile bad("{ this is not valid GeoJSON ]");
    auto planner = makePlanner(bad.path());
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, PlanFails_WhenGraphFileEmpty)
{
    auto planner = makePlanner("");   // default_value of graph_file
    astar::GridMap grid(10, 10);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 5.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, ConfigureSucceeds_LoadsValidGraph)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    // A loaded, non-empty graph must let plan() proceed past the
    // "no graph cached" early-out.
    EXPECT_TRUE(planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view).has_value());
}

// =========================================================================
// Dijkstra routing over the topological graph
// =========================================================================

TEST(RoutePathSearch, PlanSucceeds_LinearGraph_EndpointsMatchNodes)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // Start of path snaps near node A (0,0); end is exactly node C (10,0).
    EXPECT_LT(dist(result->poses.front().position.x, result->poses.front().position.y, 0.0, 0.0), 1.0);
    EXPECT_NEAR(result->poses.back().position.x, 10.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.0,  1e-6);
}

TEST(RoutePathSearch, PlanFails_DisconnectedGraph_NoRouteBetweenComponents)
{
    TempFile graph(kDisconnectedGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(40, 10);
    TestMapView view(grid, 1.0);

    // Start snaps into the {0,1} component, goal into the {2,3} component —
    // Dijkstra on the graph must find no path and return nullopt.
    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 22.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, PlanPrefersLowerCost_OverFixedExpensiveDirectEdge)
{
    TempFile graph(kCostOverrideGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.0);  // node-centres only
    astar::GridMap grid(20, 20);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 10.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // The fixed-cost direct edge (cost=100) is far more expensive than the
    // detour via node 2 (~20.8), so the route must visit node 2 -> 3 waypoints.
    ASSERT_EQ(result->poses.size(), 3u);
    EXPECT_NEAR(result->poses[0].position.x, 0.0, 1e-6);
    EXPECT_NEAR(result->poses[0].position.y, 0.0, 1e-6);
    EXPECT_NEAR(result->poses[1].position.x, 0.0, 1e-6);
    EXPECT_NEAR(result->poses[1].position.y, 8.0, 1e-6);
    EXPECT_NEAR(result->poses[2].position.x, 10.0, 1e-6);
    EXPECT_NEAR(result->poses[2].position.y, 0.0, 1e-6);
}

TEST(RoutePathSearch, Plan_SingleNodeGraph_StartEqualsGoal_ReturnsOnePoint)
{
    TempFile graph(kIsolatedNodeGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    // Both start and goal must snap to the same (only) node.
    auto result = planner.plan(makeCtx(5.6, 5.6, 5.4, 5.4), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->poses.size(), 1u);
    EXPECT_NEAR(result->poses.front().position.x, 5.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 5.5, 1e-6);
}

// =========================================================================
// Output shape: densification + world coordinates
// =========================================================================

TEST(RoutePathSearch, Plan_Densification_SpacingMatchesPathDensity)
{
    TempFile graph(kLinearGraph);
    constexpr double kDensity = 1.0;
    auto planner = makePlanner(graph.path(), kDensity);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 3u);

    // Interior segment spacing should track the configured density closely
    // (the final segment may be shorter to land exactly on the goal node).
    for (size_t i = 1; i + 1 < result->poses.size(); ++i) {
        const double seg = dist(result->poses[i - 1].position.x, result->poses[i - 1].position.y,
                                result->poses[i].position.x,     result->poses[i].position.y);
        EXPECT_NEAR(seg, kDensity, 0.2);
    }
}

TEST(RoutePathSearch, Plan_ZeroDensity_ReturnsNodeCentresOnly)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.0);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // Route A -> B -> C: exactly the three node centres, no interpolation.
    ASSERT_EQ(result->poses.size(), 3u);
    EXPECT_NEAR(result->poses[0].position.x, 0.0,  1e-6);
    EXPECT_NEAR(result->poses[1].position.x, 5.0,  1e-6);
    EXPECT_NEAR(result->poses[2].position.x, 10.0, 1e-6);
}

TEST(RoutePathSearch, Plan_OutputPoses_HaveValidOrientation)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), 0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    for (const auto& pose : result->poses) {
        // RoutePathSearch packs an identity orientation; a higher layer
        // assigns headings later. Check it is at least a valid quaternion.
        const double n = std::hypot(std::hypot(pose.orientation.x, pose.orientation.y),
                                    std::hypot(pose.orientation.z, pose.orientation.w));
        EXPECT_NEAR(n, 1.0, 1e-6);
    }
}

TEST(RoutePathSearch, Plan_TotalLength_RoughlyMatchesGraphDistance)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), 0.25);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // A(0,0) -> B(5,0) -> C(10,0): 10 m of graph distance.
    EXPECT_NEAR(pathLength(result->poses), 10.0, 0.5);
}

// =========================================================================
// Obstacle-aware pose -> route-node snapping (findNearestNodeIndex)
// =========================================================================

TEST(RoutePathSearch, Plan_SnapsToReachableNode_SkipsObstacleEnclosedNode)
{
    TempFile graph(kSnapGraph);
    auto planner = makePlanner(graph.path());

    // Open N8-connected grid except node P's cell (2,2), which is walled in
    // on all sides — geometrically closer to the robot/goal than Q, but
    // unreachable without crossing obstacles.
    astar::GridMap grid(12, 12);
    enclose(grid, 2, 2);
    TestMapView view(grid, 1.0);

    // Both start and goal sit near the enclosed node P (2.5,2.5) — the only
    // reachable node is Q (6.5,0.5), so the obstacle-aware BFS must snap
    // both endpoints to Q, collapsing the route to a single point there.
    auto result = planner.plan(makeCtx(0.6, 0.6, 0.7, 0.7), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->poses.size(), 1u);
    EXPECT_NEAR(result->poses.front().position.x, 6.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);
}

TEST(RoutePathSearch, PlanFails_WhenNoNodeIsReachableFromRobot)
{
    TempFile graph(kIsolatedNodeGraph);
    auto planner = makePlanner(graph.path());

    // Wall the only node's cell off completely — the obstacle-aware search
    // can never find a cell containing a graph node.
    astar::GridMap grid(12, 12);
    enclose(grid, 5, 5);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 0.5, 0.5), view).has_value());
}

TEST(RoutePathSearch, PlanFails_WhenStartCellItselfIsBlocked)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path());

    astar::GridMap grid(12, 12);
    grid.setObstacle(0, 0, true);   // robot starts inside an obstacle cell
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view).has_value());
}
