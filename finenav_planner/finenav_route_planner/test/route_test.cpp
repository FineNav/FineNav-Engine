// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// RoutePathSearch (topological route planning) test suite.
//
// Covers:
//   - GeoJSON graph loading via updateGraph() (valid / missing / malformed)
//   - Dijkstra routing on the topological graph (reachable / disconnected /
//     fixed-cost-override route selection / start==goal snapping)
//   - Obstacle-aware pose-to-node snapping (snapToNearestNode), including
//     skipping a geometrically-closer but unreachable node
//   - Route trimming (snapToRoute): the robot enters the route at its
//     nearest reachable route point and the path leaves the route at the
//     point nearest to the goal — no backtracking to node centres at
//     either end, no matter how many nodes the robot has already passed
//   - Connector paths: obstacle-aware grid paths from robot/goal pose to
//     the route entry/exit points are stitched into the output
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
    p.path_density = path_density;
    planner.configure(p);

    if (!graph_file.empty()) {
        std::ifstream f(graph_file);
        if (f.is_open()) {
            std::ostringstream buf;
            buf << f.rdbuf();
            planner.updateGraph(buf.str());
        }
    }
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

// Two nodes A(0,0) and B(10,0) connected by a dense polyline edge
// with intermediate points at every 2 m.  This enables path-point snapping.
const char* kDensePathGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[2,0],[4,0],[6,0],[8,0],[10,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[8,0],[6,0],[4,0],[2,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

// Three-node chain A(0,0) — B(10,0) — C(20,0) with dense polylines.
// Used to test U-turn prevention: robot near mid-path should enter the
// graph at the nearest path point and go forward toward the goal instead
// of backtracking to a node.
const char* kThreeNodeChainGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,0]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[20,0]},"properties":{"id":2,"frame":"C"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[2,0],[4,0],[6,0],[8,0],[10,0]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[8,0],[6,0],[4,0],[2,0],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,0],[12,0],[14,0],[16,0],[18,0],[20,0]]},"properties":{"id":102,"startid":1,"endid":2}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[20,0],[18,0],[16,0],[14,0],[12,0],[10,0]]},"properties":{"id":103,"startid":2,"endid":1}}
  ]
}
)json";

// A single edge with a dense polyline: A(0,0) → B(10,10) via intermediate points.
// Tests path snapping on a diagonal edge.
const char* kDiagonalDenseGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[0,0]},"properties":{"id":0}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[10,10]},"properties":{"id":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[0,0],[2,2],[4,4],[6,6],[8,8],[10,10]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[10,10],[8,8],[6,6],[4,4],[2,2],[0,0]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

// U-shaped edge: A(2,2) → B(2,8), but the polyline detours east around
// (8,·).  The A→B chord points north while the polyline leaves A heading
// east — chord-based direction tests misjudge a robot standing east of A.
const char* kUShapeEdgeGraph = R"json(
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","geometry":{"type":"Point","coordinates":[2,2]},"properties":{"id":0,"frame":"A"}},
    {"type":"Feature","geometry":{"type":"Point","coordinates":[2,8]},"properties":{"id":1,"frame":"B"}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[2,2],[3,2],[4,2],[5,2],[6,2],[7,2],[8,2],[8,3],[8,4],[8,5],[8,6],[8,7],[8,8],[7,8],[6,8],[5,8],[4,8],[3,8],[2,8]]},"properties":{"id":100,"startid":0,"endid":1}},
    {"type":"Feature","geometry":{"type":"LineString","coordinates":[[2,8],[3,8],[4,8],[5,8],[6,8],[7,8],[8,8],[8,7],[8,6],[8,5],[8,4],[8,3],[8,2],[7,2],[6,2],[5,2],[4,2],[3,2],[2,2]]},"properties":{"id":101,"startid":1,"endid":0}}
  ]
}
)json";

}  // namespace

// =========================================================================
// route::findRoute node-sequence out-param
// =========================================================================

TEST(RouteSearch, FindRoute_FillsNodeSequence)
{
    auto g = route::loadFromJson(kLinearGraph);
    ASSERT_TRUE(g.has_value());

    std::vector<size_t> seq;
    auto poses = route::findRoute(*g, g->indexOf(0), g->indexOf(2), 0.5, &seq);
    ASSERT_TRUE(poses.has_value());
    ASSERT_EQ(seq.size(), 3u);
    EXPECT_EQ(seq[0], g->indexOf(0));
    EXPECT_EQ(seq[1], g->indexOf(1));
    EXPECT_EQ(seq[2], g->indexOf(2));

    // start == goal → single-element sequence
    auto same = route::findRoute(*g, g->indexOf(1), g->indexOf(1), 0.5, &seq);
    ASSERT_TRUE(same.has_value());
    ASSERT_EQ(seq.size(), 1u);
    EXPECT_EQ(seq[0], g->indexOf(1));
}

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

TEST(RoutePathSearch, PlanSucceeds_LinearGraph_EndpointsNearRobotAndGoal)
{
    // The output includes obstacle-aware connector paths on both sides.
    // Route trimming enters at the robot's nearest route point (0.5, 0)
    // and leaves at the goal's nearest route point (9.5, 0) — neither
    // node A nor node C is visited.
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point near robot (0.5, 0.5), not near node A (0, 0)
    EXPECT_NEAR(result->poses.front().position.x, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // Last point near goal (9.5, 0.5), not node C (10, 0)
    EXPECT_NEAR(result->poses.back().position.x, 9.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);

    // The route passes through B; A and C are trimmed away.
    bool has_node_a = false, has_node_b = false, has_node_c = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 0.0, p.position.y - 0.0) < 0.01) has_node_a = true;
        if (std::hypot(p.position.x - 5.0, p.position.y - 0.0) < 0.01) has_node_b = true;
        if (std::hypot(p.position.x - 10.0, p.position.y - 0.0) < 0.01) has_node_c = true;
    }
    EXPECT_FALSE(has_node_a);
    EXPECT_TRUE(has_node_b);
    EXPECT_FALSE(has_node_c);
}

TEST(RoutePathSearch, Plan_DisconnectedGraph_ReSnapsToConnectedComponent)
{
    // Two isolated pairs: {0,1} near the origin, {2,3} far away.
    // Start initially snaps into {0,1}, goal into {2,3}.  The connectivity
    // fix detects the mismatch, identifies the smaller component (goal side,
    // only 2 nodes), and re-snaps the goal into the start component's
    // node 1 — producing a valid plan via a long connector.
    TempFile graph(kDisconnectedGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(40, 10);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 22.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // The plan succeeds: note the route graph component is {0,1} only, so
    // Dijkstra on just 2 nodes returns a short network segment; the bulk
    // of the journey is the long connector from the goal's re-snap.
    EXPECT_NEAR(result->poses.front().position.x, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.x, 22.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.5, 1e-6);
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
    // detour via node 2 (~20.8), so the route must visit node 0 → 2 → 1 in
    // order.  With path_density = 0 the route contains node centres only,
    // so trimming operates at node granularity and all three nodes appear.
    ASSERT_GE(result->poses.size(), 3u);

    // Find the positions of the route nodes in the output
    auto findNode = [&](double x, double y) -> std::optional<size_t> {
        for (size_t i = 0; i < result->poses.size(); ++i)
            if (std::hypot(result->poses[i].position.x - x,
                           result->poses[i].position.y - y) < 1e-6)
                return i;
        return std::nullopt;
    };

    auto i0 = findNode(0.0, 0.0);
    auto i2 = findNode(0.0, 8.0);
    auto i1 = findNode(10.0, 0.0);
    ASSERT_TRUE(i0.has_value());
    ASSERT_TRUE(i2.has_value());
    ASSERT_TRUE(i1.has_value());
    EXPECT_LT(*i0, *i2);
    EXPECT_LT(*i2, *i1);
}

TEST(RoutePathSearch, Plan_SingleNodeGraph_StartEqualsGoal_ConnectorsBothSides)
{
    // Both start and goal snap to the same node.  The output now includes
    // connectors on both sides: robot → node → goal.
    TempFile graph(kIsolatedNodeGraph);
    auto planner = makePlanner(graph.path());
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.6, 5.6, 5.4, 5.4), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point is the robot pose
    EXPECT_NEAR(result->poses.front().position.x, 5.6, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 5.6, 1e-6);

    // Last point is the goal pose
    EXPECT_NEAR(result->poses.back().position.x, 5.4, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 5.4, 1e-6);

    // The node (5.5, 5.5) appears somewhere in the middle
    bool has_node = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 5.5, p.position.y - 5.5) < 1e-6)
            has_node = true;
    }
    EXPECT_TRUE(has_node);
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

    // Interior segment spacing should track the configured density closely.
    // Skip the start/goal connector segments: those have non-zero y (robot
    // at y=0.5) and/or diagonal steps (~0.707 m), while route edges lie on y=0.
    for (size_t i = 1; i + 1 < result->poses.size(); ++i) {
        const auto& a = result->poses[i - 1].position;
        const auto& b = result->poses[i].position;
        // Connector segments have y ≠ 0 or x outside the route extent [0, 10].
        const bool a_on_route = (std::abs(a.y) < 1e-6 && a.x >= 0.0 && a.x <= 10.0);
        const bool b_on_route = (std::abs(b.y) < 1e-6 && b.x >= 0.0 && b.x <= 10.0);
        if (!a_on_route || !b_on_route) continue;
        const double seg = dist(a.x, a.y, b.x, b.y);
        EXPECT_NEAR(seg, kDensity, 0.2);
    }
}

TEST(RoutePathSearch, Plan_ZeroDensity_IncludesNodeCentresAndConnectors)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.0);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // With zero density the route contains node centres only, so trimming
    // operates at node granularity: A(0,0), B(5,0), C(10,0) all appear in
    // order.  (Mid-edge entry/exit requires path_density > 0.)
    auto findNode = [&](double x, double y) -> std::optional<size_t> {
        for (size_t i = 0; i < result->poses.size(); ++i)
            if (std::hypot(result->poses[i].position.x - x,
                           result->poses[i].position.y - y) < 1e-4)
                return i;
        return std::nullopt;
    };

    auto iA = findNode(0.0, 0.0);
    auto iB = findNode(5.0, 0.0);
    auto iC = findNode(10.0, 0.0);
    ASSERT_TRUE(iA.has_value());
    ASSERT_TRUE(iB.has_value());
    ASSERT_TRUE(iC.has_value());
    EXPECT_LT(*iA, *iB);
    EXPECT_LT(*iB, *iC);
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

TEST(RoutePathSearch, Plan_TotalLength_CoversRobotToGoal)
{
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), 0.25);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // Trimming enters at (0.5,0) and leaves at (9.5,0):
    // 0.5 (robot → entry) + 9.0 (route) + 0.5 (exit → goal) ≈ 10 m.
    const double len = pathLength(result->poses);
    EXPECT_GT(len, 9.0);
    EXPECT_LT(len, 12.0);
}

// =========================================================================
// Obstacle-aware pose -> route-node snapping (findNearestNodeIndex)
// =========================================================================

TEST(RoutePathSearch, Plan_SnapsToReachableNode_IncludesConnectorPath)
{
    // Node P (2.5,2.5) is enclosed by obstacles — geometrically closer but
    // unreachable.  Both robot and goal snap to node Q (6.5,0.5).
    // The output must include the connector path from robot to Q,
    // which proves the obstacle-aware search worked.
    TempFile graph(kSnapGraph);
    auto planner = makePlanner(graph.path());

    // Open N8-connected grid except node P's cell (2,2), which is walled in.
    astar::GridMap grid(12, 12);
    enclose(grid, 2, 2);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.6, 0.6, 0.7, 0.7), view);
    ASSERT_TRUE(result.has_value());

    // Output now includes connectors — at least 2 points (start connector + goal connector).
    ASSERT_GE(result->poses.size(), 2u);

    // First point is robot pose
    EXPECT_NEAR(result->poses.front().position.x, 0.6, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.6, 1e-6);

    // Last point is goal pose
    EXPECT_NEAR(result->poses.back().position.x, 0.7, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.7, 1e-6);

    // Node Q (6.5, 0.5) appears somewhere (the reachable node)
    bool has_q = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 6.5, p.position.y - 0.5) < 1e-4)
            has_q = true;
    }
    EXPECT_TRUE(has_q);

    // Node P (2.5, 2.5) must NOT appear (it's unreachable)
    bool has_p = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 2.5, p.position.y - 2.5) < 1e-4)
            has_p = true;
    }
    EXPECT_FALSE(has_p);
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

// =========================================================================
// Path-point snapping (new: snap to intermediate polyline points)
// =========================================================================

TEST(RoutePathSearch, Plan_StartPrune_ReplacesFirstNodeWithPathPoint_WhenRobotPastIt)
{
    // Graph: A(0,0) — B(10,0), dense polyline.  Goal on node B.
    // Robot at (6, 0.5) — past node A, projecting positively onto A→B.
    // Dot-product prune: robot should skip node A and snap to a path point
    // on A→B, then route to B.  Path must NOT backtrack to A at x=0.
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(6.0, 0.5, 10.0, 0.0), view);  // goal on node B
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 6.0, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // Last point on node B
    EXPECT_NEAR(result->poses.back().position.x, 10.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.0, 1e-6);

    // Must not backtrack to node A at x=0
    for (const auto& p : result->poses)
        EXPECT_GT(p.position.x, 1.0) << "Backtracked to A: x=" << p.position.x;

    // Path should be short (robot enters at a nearby path point, not node A)
    const double len = pathLength(result->poses);
    EXPECT_LT(len, 8.0);
}

TEST(RoutePathSearch, Plan_ShortConnector_WhenRobotNearDensePathFarFromNodes)
{
    // Robot at (5, 0.5) — geometrically near the path midpoint, far from nodes.
    // Goal at (9.5, 0.5) — near node B.
    // Without path-snapping, robot would walk to node A (0,0) or B (10,0) first.
    // With path-snapping, it should enter at the nearest path point (~5,0).
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.0, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 3u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 5.0, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);

    // The connector should be short: robot goes to a path point near (5, 0),
    // not all the way to node A at (0, 0) or node B at (10, 0).
    // Verify that a point near (5, 0) appears early in the path.
    bool has_midpoint_entry = false;
    for (size_t i = 0; i < std::min(size_t(5), result->poses.size()); ++i) {
        const auto& p = result->poses[i];
        if (std::hypot(p.position.x - 5.0, p.position.y) < 1.0)
            has_midpoint_entry = true;
    }
    EXPECT_TRUE(has_midpoint_entry);

    // Total path should be significantly shorter than node-only snapping
    // (which would be ~0.7m robot→A + 10m A→B + 0.7m B→goal ≈ 11.4m)
    const double len = pathLength(result->poses);
    EXPECT_LT(len, 8.0);
}

TEST(RoutePathSearch, Plan_ThreeNodeChain_NoBacktrack_GoalOnNode)
{
    // Graph: A(0,0) — B(10,0) — C(20,0).
    // Robot at (9.5, 1) near B, goal on node C at (20, 0).
    // The robot's nearest route point is node B itself (route B→C starts
    // there), so the full route B→C is kept.  Must NOT backtrack to A.
    TempFile graph(kThreeNodeChainGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(22, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(9.5, 1.0, 20.0, 0.0), view);  // goal on node C
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 3u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 9.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 1.0, 1e-6);

    // Last point on node C
    EXPECT_NEAR(result->poses.back().position.x, 20.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.0, 1e-6);

    // Must not backtrack to node A at x=0
    for (const auto& p : result->poses)
        EXPECT_GT(p.position.x, 1.0) << "Backtracked to A: x=" << p.position.x;

    // Path should visit node B
    bool has_b = false;
    for (const auto& p : result->poses)
        if (std::hypot(p.position.x - 10.0, p.position.y) < 1e-4) has_b = true;
    EXPECT_TRUE(has_b);

    // Reasonable path length
    const double len = pathLength(result->poses);
    EXPECT_GT(len, 8.0);
    EXPECT_LT(len, 18.0);
}

TEST(RoutePathSearch, Plan_SnapsToNode_WhenNodeIsCloserThanAnyPathPoint)
{
    // Robot at (0.5, 0.5), goal at (9.5, 0.5) on A(0,0)→B(10,0).
    // Dot-product prune: robot past A (dot>0) → snaps to a path point on A→B.
    // Node A should NOT appear in the output.
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 9.5, 0.5), view);
    ASSERT_TRUE(result.has_value());

    // Node A not visited — prune replaced it with a path point
    bool has_node_a = false;
    for (const auto& p : result->poses) {
        if (std::hypot(p.position.x - 0.0, p.position.y - 0.0) < 1e-4)
            has_node_a = true;
    }
    EXPECT_FALSE(has_node_a);
}

TEST(RoutePathSearch, Plan_DiagonalDenseEdge_SnapsToPathPoint)
{
    // Robot at (5.5, 4.5) — near the diagonal path between (0,0) and (10,10).
    // Goal at (9.5, 9.5) — near the end node.
    // Path-snapping should enter the graph at the nearest diagonal path point.
    TempFile graph(kDiagonalDenseGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.5, 4.5, 9.5, 9.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    // First point near robot
    EXPECT_NEAR(result->poses.front().position.x, 5.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 4.5, 1e-6);

    // Last point near goal
    EXPECT_NEAR(result->poses.back().position.x, 9.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 9.5, 1e-6);

    // Should include points on the diagonal (y ≈ x)
    bool has_diag_point = false;
    for (const auto& p : result->poses) {
        if (std::abs(p.position.x - p.position.y) < 0.2 &&
            p.position.x > 0.0 && p.position.x < 10.0)
            has_diag_point = true;
    }
    EXPECT_TRUE(has_diag_point);
}

TEST(RoutePathSearch, Plan_StartTrim_StraightEdge_NoBacktrackToNode)
{
    // kLinearGraph edges have no intermediate polyline points, but the
    // densified route curve still provides trim points everywhere.  Robot
    // at (6.5, 2.5) is past node B on edge B→C: the route must enter near
    // (6.5, 0) instead of backtracking to node B (5, 0).
    //
    // Regression: an earlier implementation failed to split the edge at a
    // projected (non-vertex) snap point and returned nullopt in this case.
    TempFile graph(kLinearGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(6.5, 2.5, 10.0, 0.0), view);  // goal on node C
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    EXPECT_NEAR(result->poses.front().position.x, 6.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 2.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.x, 10.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 0.0, 1e-6);

    // No backtracking to node B at (5, 0)
    for (const auto& p : result->poses)
        EXPECT_GT(p.position.x, 5.5) << "Backtracked toward B: x=" << p.position.x;

    // robot→(6.5,0) ≈ 2.5, (6.5,0)→C = 3.5 → total ≈ 6 m
    const double len = pathLength(result->poses);
    EXPECT_LT(len, 7.5);
}

TEST(RoutePathSearch, Plan_StartTrim_CurvedEdge_EntersAlongPolyline)
{
    // Robot at (5, 1.5): the A→B chord points north, but the polyline
    // leaves A heading EAST — the robot is already 3 m along the edge.
    // The trim must enter the route at (5, 2) instead of backtracking to
    // A (2, 2).
    //
    // Regression: a chord-based direction test misjudged this case and the
    // robot walked back to A before re-passing its own position.
    TempFile graph(kUShapeEdgeGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(5.0, 1.5, 2.0, 8.5), view);  // goal near node B
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    EXPECT_NEAR(result->poses.front().position.x, 5.0, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 1.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.x, 2.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 8.5, 1e-6);

    // Node A must NOT appear — the route enters mid-edge at (5, 2).
    for (const auto& p : result->poses)
        EXPECT_GT(std::hypot(p.position.x - 2.0, p.position.y - 2.0), 0.5)
            << "Backtracked to A: (" << p.position.x << ", " << p.position.y << ")";

    // The path still follows the U detour (passes the east corridor).
    bool has_east_corridor = false;
    for (const auto& p : result->poses)
        if (std::hypot(p.position.x - 8.0, p.position.y - 5.0) < 0.3)
            has_east_corridor = true;
    EXPECT_TRUE(has_east_corridor);
}

TEST(RoutePathSearch, Plan_GoalTrim_LeavesRouteAtNearestPoint)
{
    // Goal at (8, 1.5) is alongside the route A(0,0)→B(10,0), well before
    // node B.  The goal-side trim must leave the route near (8, 0) and go
    // straight to the goal — NOT continue to node B and backtrack.
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path(), /*path_density=*/0.5);
    astar::GridMap grid(12, 12);
    TestMapView view(grid, 1.0);

    auto result = planner.plan(makeCtx(0.5, 0.5, 8.0, 1.5), view);
    ASSERT_TRUE(result.has_value());
    ASSERT_GE(result->poses.size(), 2u);

    EXPECT_NEAR(result->poses.front().position.x, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.front().position.y, 0.5, 1e-6);
    EXPECT_NEAR(result->poses.back().position.x, 8.0, 1e-6);
    EXPECT_NEAR(result->poses.back().position.y, 1.5, 1e-6);

    // Node B (10, 0) must NOT appear — the route is cut near (8, 0).
    for (const auto& p : result->poses)
        EXPECT_LT(p.position.x, 9.0)
            << "Overshot toward B: (" << p.position.x << ", " << p.position.y << ")";
}

TEST(RoutePathSearch, PlanFails_WhenNoGraphPointIsReachable)
{
    // Same as PlanFails_WhenNoNodeIsReachableFromRobot — but with a graph that
    // also has path points.  All are walled off, so the search must still fail.
    TempFile graph(kDensePathGraph);
    auto planner = makePlanner(graph.path());

    // Wall off the entire graph area
    astar::GridMap grid(12, 12);
    for (int x = 0; x < 11; ++x)
        for (int y = 0; y < 2; ++y)
            grid.setObstacle(x, y, true);
    TestMapView view(grid, 1.0);

    EXPECT_FALSE(planner.plan(makeCtx(5.0, 5.0, 10.0, 5.0), view).has_value());
}
