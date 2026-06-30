// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// MPPI controller benchmark — uses public plan() API, zero production code changes.

#include <benchmark/benchmark.h>
#include <memory>

#include "finenav_mppi_controller/controller.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"
#include "finenav_core/plan/plan_context.hpp"
#include "finenav_core/plan/data_packet.hpp"
#include "finenav_msgs/msg/robot_state.hpp"
#include "bench_map_view.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RosGuard {
public:
    RosGuard()  { rclcpp::init(0, nullptr); }
    ~RosGuard() { rclcpp::shutdown(); }
};
static RosGuard g_ros;

// ────────────────────────────────────────────────────────────
static finenav_mppi_controller::Params makeParams(int batch = 3000, int steps = 30, int threads = 8) {
    finenav_mppi_controller::Params p{};
    p.time_steps  = static_cast<unsigned int>(steps);
    p.model_dt    = 0.05;
    p.batch_size  = static_cast<unsigned int>(batch);
    p.iteration_count = 1;
    p.temperature = 0.3f;   p.gamma = 0.015f;
    p.vx_max = 1.0f;  p.vx_min = 0.0f;  p.vy_max = 0.0f;  p.wz_max = 2.0f;
    p.vx_std = 0.5f;  p.vy_std = 0.2f;  p.wz_std = 0.8f;
    p.motion_model = "DiffDrive";
    p.num_threads = threads;

    p.ConstraintCritic.enabled = true;       p.ConstraintCritic.cost_power = 1;  p.ConstraintCritic.cost_weight = 4.0;
    p.CostCritic.enabled = true;             p.CostCritic.cost_power = 1;        p.CostCritic.cost_weight = 3.81;
    p.CostCritic.critical_cost = 100.0;      p.CostCritic.collision_cost = 1e6f; p.CostCritic.near_goal_distance = 1.0;
    p.GoalCritic.enabled = true;             p.GoalCritic.cost_power = 1;        p.GoalCritic.cost_weight = 5.0;   p.GoalCritic.threshold_to_consider = 0.5;
    p.GoalAngleCritic.enabled = true;        p.GoalAngleCritic.cost_power = 1;   p.GoalAngleCritic.cost_weight = 3.0;  p.GoalAngleCritic.threshold_to_consider = 0.5;
    p.PathAlignCritic.enabled = true;        p.PathAlignCritic.cost_power = 1;   p.PathAlignCritic.cost_weight = 14.0;
    p.PathAlignCritic.max_path_occupancy_ratio = 0.05;  p.PathAlignCritic.trajectory_point_step = 4;
    p.PathAlignCritic.threshold_to_consider = 0.5;  p.PathAlignCritic.offset_from_furthest = 20;  p.PathAlignCritic.use_path_orientations = false;
    p.PathFollowCritic.enabled = true;       p.PathFollowCritic.cost_power = 1;  p.PathFollowCritic.cost_weight = 5.0;
    p.PathFollowCritic.offset_from_furthest = 15;  p.PathFollowCritic.threshold_to_consider = 0.5;
    p.PathAngleCritic.enabled = true;        p.PathAngleCritic.cost_power = 1;   p.PathAngleCritic.cost_weight = 2.0;
    p.PathAngleCritic.offset_from_furthest = 15;  p.PathAngleCritic.threshold_to_consider = 0.5;  p.PathAngleCritic.max_angle_to_furthest = 1.0;
    p.PreferForwardCritic.enabled = false;   p.VelocityDeadbandCritic.enabled = false;
    p.VelocityDeadbandCritic.deadband_velocities = {0.05, 0.05, 0.2};
    p.TwirlingCritic.enabled = false;
    return p;
}

static geometry_msgs::msg::Pose makePose(double x, double y) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;  pose.position.y = y;  pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

// ────────────────────────────────────────────────────────────
// Single benchmark runner — create fresh controller per test
// ────────────────────────────────────────────────────────────
static void runBench(benchmark::State& st,
                     bench::Scenario sc, bool fp,
                     int batch, int steps,
                     double rx, double ry, double heading, double path_len,
                     int cost_stride = 2,
                     int num_threads = 8,
                     bool precompute_noise = true) {
    auto map_view = std::make_unique<bench::BenchMapView>(sc);
    map_view->setFootprint(fp);

    auto params = makeParams(batch, steps, num_threads);
    params.CostCritic.trajectory_point_step = cost_stride;
    params.regenerate_noises = precompute_noise;

    auto controller = std::make_unique<nav2_mppi_controller::MPPIController>();
    controller->configure(params);

    // Build a PlanningContext with a straight reference trajectory
    using InputTraj = finenav::InputDataT<nav2_mppi_controller::MPPIController>;
    InputTraj ref_traj;
    double dx = std::cos(heading), dy = std::sin(heading);
    for (int i = 0; i < 50; ++i) {
        double f = static_cast<double>(i) / 49.0;
        ref_traj.poses.push_back(makePose(rx + f * path_len * dx, ry + f * path_len * dy));
    }

    finenav::PlanningContext<nav2_mppi_controller::MPPIController> ctx{
        .robot_state = finenav_msgs::msg::RobotState{},
        .goal_pose   = makePose(rx + path_len * dx, ry + path_len * dy),
        .ref_traj    = std::move(ref_traj)};

    for (auto _ : st) {
        try {
            controller->plan(ctx, *map_view);
        } catch (const std::exception&) {
            controller->reset();
        }
    }
    st.SetItemsProcessed(st.iterations());

    // Destroy controller before map_view
    controller.reset();
    map_view.reset();
}

// ────────────────────────────────────────────────────────────
static void BM_Open(benchmark::State& st) {
    runBench(st, bench::Scenario::kOpen, true, 3000, 30, 0,0, 0, 10);
}
BENCHMARK(BM_Open)->Unit(benchmark::kMillisecond)->Name("mppi/01_open");

static void BM_WallAhead(benchmark::State& st) {
    runBench(st, bench::Scenario::kWallAhead, true, 3000, 30, 0,0, 0, 10);
}
BENCHMARK(BM_WallAhead)->Unit(benchmark::kMillisecond)->Name("mppi/02_wall_ahead");

static void BM_Inflation(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 3000, 30, 0,0, 0, 10);
}
BENCHMARK(BM_Inflation)->Unit(benchmark::kMillisecond)->Name("mppi/03_inflation_field");

static void BM_Minefield(benchmark::State& st) {
    runBench(st, bench::Scenario::kMinefield, true, 3000, 30, 5,0, M_PI, 10);
}
BENCHMARK(BM_Minefield)->Unit(benchmark::kMillisecond)->Name("mppi/04_minefield");

static void BM_InflationNoFP(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, false, 3000, 30, 0,0, 0, 10);
}
BENCHMARK(BM_InflationNoFP)->Unit(benchmark::kMillisecond)->Name("mppi/05_inflation_nofp");

static void BM_MinefieldNoFP(benchmark::State& st) {
    runBench(st, bench::Scenario::kMinefield, false, 3000, 30, 5,0, M_PI, 10);
}
BENCHMARK(BM_MinefieldNoFP)->Unit(benchmark::kMillisecond)->Name("mppi/06_minefield_nofp");

static void BM_Inflation1k(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 1000, 30, 0,0, 0, 10);
}
BENCHMARK(BM_Inflation1k)->Unit(benchmark::kMillisecond)->Name("mppi/07_inflation_b1k");

static void BM_Minefield1k(benchmark::State& st) {
    runBench(st, bench::Scenario::kMinefield, true, 1000, 30, 5,0, M_PI, 10);
}
BENCHMARK(BM_Minefield1k)->Unit(benchmark::kMillisecond)->Name("mppi/08_minefield_b1k");

// ── high-load stress (batch=10000, steps=50) ──
static void BM_Open10k50(benchmark::State& st) {
    runBench(st, bench::Scenario::kOpen, true, 10000, 50, 0,0, 0, 10);
}
BENCHMARK(BM_Open10k50)->Unit(benchmark::kMillisecond)->Name("mppi/09_open_b10k_s50");

static void BM_Inflation10k50(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 10000, 50, 0,0, 0, 10);
}
BENCHMARK(BM_Inflation10k50)->Unit(benchmark::kMillisecond)->Name("mppi/10_inflation_b10k_s50");

// ── stride=1 (no skip) vs stride=2 (default) comparison ──
static void BM_InflationNoStride(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 3000, 30, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_InflationNoStride)->Unit(benchmark::kMillisecond)->Name("mppi/A1_inflation_stride1");

static void BM_InflationStride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 3000, 30, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_InflationStride2)->Unit(benchmark::kMillisecond)->Name("mppi/A2_inflation_stride2");

static void BM_MinefieldNoStride(benchmark::State& st) {
    runBench(st, bench::Scenario::kMinefield, true, 3000, 30, 5,0, M_PI, 10, /*stride=*/1);
}
BENCHMARK(BM_MinefieldNoStride)->Unit(benchmark::kMillisecond)->Name("mppi/A3_minefield_stride1");

static void BM_MinefieldStride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kMinefield, true, 3000, 30, 5,0, M_PI, 10, /*stride=*/2);
}
BENCHMARK(BM_MinefieldStride2)->Unit(benchmark::kMillisecond)->Name("mppi/A4_minefield_stride2");

// ── high-load stride comparison (batch=10000, steps=50) ──
static void BM_Inflation10k50Stride1(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 10000, 50, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_Inflation10k50Stride1)->Unit(benchmark::kMillisecond)->Name("mppi/B1_inflation_b10k_s50_stride1");

static void BM_Inflation10k50Stride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationField, true, 10000, 50, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_Inflation10k50Stride2)->Unit(benchmark::kMillisecond)->Name("mppi/B2_inflation_b10k_s50_stride2");

// ── inflation grid: continuous gradient, footprint on nearly every point ──
static void BM_InflationGridStride1(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationGrid, true, 3000, 30, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_InflationGridStride1)->Unit(benchmark::kMillisecond)->Name("mppi/C1_inflation_grid_stride1");

static void BM_InflationGridStride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationGrid, true, 3000, 30, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_InflationGridStride2)->Unit(benchmark::kMillisecond)->Name("mppi/C2_inflation_grid_stride2");

static void BM_InflationGridStride4(benchmark::State& st) {
    runBench(st, bench::Scenario::kInflationGrid, true, 3000, 30, 0,0, 0, 10, /*stride=*/4);
}
BENCHMARK(BM_InflationGridStride4)->Unit(benchmark::kMillisecond)->Name("mppi/C3_inflation_grid_stride4");

// ── OBB stress: ultra-dense pillars, inflation everywhere → OBB on nearly every point ──
static void BM_ObbStressStride1(benchmark::State& st) {
    runBench(st, bench::Scenario::kObbStress, true, 3000, 30, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_ObbStressStride1)->Unit(benchmark::kMillisecond)->Name("mppi/D1_obb_stress_stride1");

static void BM_ObbStressStride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kObbStress, true, 3000, 30, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_ObbStressStride2)->Unit(benchmark::kMillisecond)->Name("mppi/D2_obb_stress_stride2");

static void BM_ObbStressStride4(benchmark::State& st) {
    runBench(st, bench::Scenario::kObbStress, true, 3000, 30, 0,0, 0, 10, /*stride=*/4);
}
BENCHMARK(BM_ObbStressStride4)->Unit(benchmark::kMillisecond)->Name("mppi/D3_obb_stress_stride4");

static void BM_ObbStress1kStride1(benchmark::State& st) {
    runBench(st, bench::Scenario::kObbStress, true, 1000, 30, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_ObbStress1kStride1)->Unit(benchmark::kMillisecond)->Name("mppi/D4_obb_stress_b1k_stride1");

static void BM_ObbStress10k50Stride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kObbStress, true, 10000, 50, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_ObbStress10k50Stride2)->Unit(benchmark::kMillisecond)->Name("mppi/D5_obb_stress_b10k_s50");

// ── Corridor + obstacle: every point triggers OBB, most trajectories don't collide ──
static void BM_CorridorStride1(benchmark::State& st) {
    runBench(st, bench::Scenario::kCorridorObstacle, true, 3000, 30, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_CorridorStride1)->Unit(benchmark::kMillisecond)->Name("mppi/E1_corridor_obstacle_stride1");

static void BM_CorridorStride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kCorridorObstacle, true, 3000, 30, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_CorridorStride2)->Unit(benchmark::kMillisecond)->Name("mppi/E2_corridor_obstacle_stride2");

// ── Corridor high-load: batch=10000, steps=50 ──
static void BM_Corridor10k50Stride1(benchmark::State& st) {
    runBench(st, bench::Scenario::kCorridorObstacle, true, 10000, 50, 0,0, 0, 10, /*stride=*/1);
}
BENCHMARK(BM_Corridor10k50Stride1)->Unit(benchmark::kMillisecond)->Name("mppi/E3_corridor_b10k_s50_stride1");

static void BM_Corridor10k50Stride2(benchmark::State& st) {
    runBench(st, bench::Scenario::kCorridorObstacle, true, 10000, 50, 0,0, 0, 10, /*stride=*/2);
}
BENCHMARK(BM_Corridor10k50Stride2)->Unit(benchmark::kMillisecond)->Name("mppi/E4_corridor_b10k_s50_stride2");

// ═══════════════════════════════════════════════════════════════════════════════
// Thread × Precompute noise 对比（12 种配置）
//   线程数: 1, 2, 4, 8, 16, 32
//   预计算: yes (P) / no (N)
// ═══════════════════════════════════════════════════════════════════════════════

// 注册 helper — 为 (scenario, batch, steps, threads, precompute) 生成独立 benchmark
#define BENCH_CONFIG(label_prefix, sc, fp, batch, steps, rx, ry, h, plen, stride, T, P) \
  static void BM_##label_prefix##_T##T##_##P(benchmark::State& st) { \
    runBench(st, bench::Scenario::sc, fp, batch, steps, rx, ry, h, plen, stride, T, P == 1); \
  } \
  BENCHMARK(BM_##label_prefix##_T##T##_##P)->Unit(benchmark::kMillisecond) \
    ->Name("thread_precompute/" #label_prefix "/threads_" #T "/precompute_" #P);

// ── 场景: InflationField (batch=3000, steps=30) ──
#define BENCH_IF_ALL_THREADS(T) \
  BENCH_CONFIG(inf, kInflationField, true, 3000, 30, 0,0, 0, 10, 2, T, 1) \
  BENCH_CONFIG(inf, kInflationField, true, 3000, 30, 0,0, 0, 10, 2, T, 0)

BENCH_IF_ALL_THREADS(1)
BENCH_IF_ALL_THREADS(2)
BENCH_IF_ALL_THREADS(4)
BENCH_IF_ALL_THREADS(8)
BENCH_IF_ALL_THREADS(16)
BENCH_IF_ALL_THREADS(32)

// ── 场景: ObbStress (batch=3000, steps=30) ──
#define BENCH_OS_ALL_THREADS(T) \
  BENCH_CONFIG(obb, kObbStress, true, 3000, 30, 0,0, 0, 10, 2, T, 1) \
  BENCH_CONFIG(obb, kObbStress, true, 3000, 30, 0,0, 0, 10, 2, T, 0)

BENCH_OS_ALL_THREADS(1)
BENCH_OS_ALL_THREADS(2)
BENCH_OS_ALL_THREADS(4)
BENCH_OS_ALL_THREADS(8)
BENCH_OS_ALL_THREADS(16)
BENCH_OS_ALL_THREADS(32)

// ── 场景: InflationField 高负载 (batch=10000, steps=50) ──
#define BENCH_IF10k_ALL_THREADS(T) \
  BENCH_CONFIG(inf10k, kInflationField, true, 10000, 50, 0,0, 0, 10, 2, T, 1) \
  BENCH_CONFIG(inf10k, kInflationField, true, 10000, 50, 0,0, 0, 10, 2, T, 0)

BENCH_IF10k_ALL_THREADS(1)
BENCH_IF10k_ALL_THREADS(2)
BENCH_IF10k_ALL_THREADS(4)
BENCH_IF10k_ALL_THREADS(8)
BENCH_IF10k_ALL_THREADS(16)
BENCH_IF10k_ALL_THREADS(32)

// ── 场景: ObbStress 高负载 (batch=10000, steps=50) ──
#define BENCH_OS10k_ALL_THREADS(T) \
  BENCH_CONFIG(obb10k, kObbStress, true, 10000, 50, 0,0, 0, 10, 2, T, 1) \
  BENCH_CONFIG(obb10k, kObbStress, true, 10000, 50, 0,0, 0, 10, 2, T, 0)

BENCH_OS10k_ALL_THREADS(1)
BENCH_OS10k_ALL_THREADS(2)
BENCH_OS10k_ALL_THREADS(4)
BENCH_OS10k_ALL_THREADS(8)
BENCH_OS10k_ALL_THREADS(16)
BENCH_OS10k_ALL_THREADS(32)

BENCHMARK_MAIN();
