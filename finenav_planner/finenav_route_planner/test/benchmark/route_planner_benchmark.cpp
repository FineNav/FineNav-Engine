// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Performance benchmarks for A* and D* Lite planners.
//
// Build:  colcon build --packages-select finenav_route_planner
// Run:    ./build/finenav_route_planner/test/benchmark/route_planner_benchmark
// Filter: --benchmark_filter="AStar" | "DStarLite" | "Replan"

#include <benchmark/benchmark.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "grid_map.hpp"
#include "finenav_route_planner/astar_algorithm.hpp"
#include "finenav_route_planner/dstarlite_algorithm.hpp"

// =========================================================================
// Helpers
// =========================================================================

void generateObstacles(astar::GridMap& grid, int density_pct, uint64_t seed = 42) {
    if (density_pct <= 0) return;
    std::mt19937_64 rng(seed);
    int w = grid.getWidth(), h = grid.getHeight();
    int n_obs = (w * h) * density_pct / 100;

    std::uniform_int_distribution<int> dist_x(0, w - 1);
    std::uniform_int_distribution<int> dist_y(0, h - 1);

    for (int i = 0; i < n_obs; ++i) {
        int x = dist_x(rng);
        int y = dist_y(rng);
        if ((x == 0 && y == 0) || (x == w - 1 && y == h - 1)) continue;
        grid.setObstacle(x, y, true);
    }
}

// =========================================================================
// Fixture
// =========================================================================

class PlannerBench : public benchmark::Fixture {
protected:
    void SetUp(const benchmark::State& state) override {
        int size        = static_cast<int>(state.range(0));
        int density_pct = static_cast<int>(state.range(1));

        grid_ = std::make_unique<astar::GridMap>(size, size);
        generateObstacles(*grid_, density_pct);
    }

    void TearDown(const benchmark::State&) override {
        grid_.reset();
    }

    auto makeLambdas() const {
        return std::make_pair(
            [this](const astar::Vec3& c) { return grid_->isWalkableCell(c.x, c.y); },
            [this](const astar::Vec3& c) { return grid_->getCostCell(c.x, c.y); }
        );
    }

    std::unique_ptr<astar::GridMap> grid_;
};

// =========================================================================
// 1. A* — grid-scale sweep  (N8, Euclidean)
// =========================================================================

BENCHMARK_DEFINE_F(PlannerBench, BM_AStar_GridScale)(benchmark::State& state) {
    int size = static_cast<int>(state.range(0));
    auto [iw, gc] = makeLambdas();

    for (auto _ : state) {
        state.PauseTiming();
        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        algo.setGoalTolerance(0.0);
        state.ResumeTiming();

        auto path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(path);

        state.counters["path_len"] = path.has_value()
            ? static_cast<double>(path->size()) : 0.0;
    }
    state.SetComplexityN(size);
}
BENCHMARK_REGISTER_F(PlannerBench, BM_AStar_GridScale)
    ->ArgsProduct({{100, 500, 1000}, {0, 10, 30}})
    ->Unit(benchmark::kMillisecond)
    ->Complexity();

// =========================================================================
// 2. A* — heuristic comparison  (fixed 500² grid, N8)
// =========================================================================

static void BM_AStar_Heuristic(benchmark::State& state) {
    int size = 500;
    astar::GridMap grid(size, size);

    auto is_walkable = [&grid](const astar::Vec3& c) {
        return grid.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [&grid](const astar::Vec3& c) {
        return grid.getCostCell(c.x, c.y);
    };

    astar::Heuristic heur;
    switch (state.range(0)) {
        case 0: heur = astar::Heuristic::Euclidean;  break;
        case 1: heur = astar::Heuristic::Diagonal;   break;
        case 2: heur = astar::Heuristic::Manhattan;  break;
        case 3: heur = astar::Heuristic::Zero;       break;
        default: heur = astar::Heuristic::Euclidean;  break;
    }

    for (auto _ : state) {
        state.PauseTiming();
        astar::AStarAlgorithm algo(is_walkable, get_cost);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(heur);
        algo.setGoalTolerance(0.0);
        state.ResumeTiming();

        auto path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(path);

        state.counters["path_len"] = path.has_value()
            ? static_cast<double>(path->size()) : 0.0;
    }
}
BENCHMARK(BM_AStar_Heuristic)
    ->DenseRange(0, 3, 1)
    ->ArgName("heuristic")
    ->Unit(benchmark::kMillisecond);

// =========================================================================
// 3. D* Lite — initial computeShortestPath  (cold start)
// =========================================================================

BENCHMARK_DEFINE_F(PlannerBench, BM_DStarLite_InitialPlan)(benchmark::State& state) {
    int size = static_cast<int>(state.range(0));
    auto [iw, gc] = makeLambdas();

    for (auto _ : state) {
        state.PauseTiming();
        astar::DStarLiteAlgorithm engine(size, size, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.initialize({0, 0, 0}, {size - 1, size - 1, 0});
        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        state.counters["path_len"] = static_cast<double>(path.size());
    }
    state.SetComplexityN(state.range(0));
}
BENCHMARK_REGISTER_F(PlannerBench, BM_DStarLite_InitialPlan)
    ->ArgsProduct({{100, 300, 500}, {0, 10, 30}})
    ->Unit(benchmark::kMillisecond)
    ->MinTime(0.5)
    ->Complexity();

// =========================================================================
// 4. ★  D* Lite vs A* — replan after new obstacles block the path
//
//    Grid with 30% random obstacles → expensive initial planning.
//    Then block N MORE cells on the found path.
//    D* Lite replan() incrementally fixes only the affected region.
//    A* must re-run findPath() from scratch over the full grid.
//    This is the scenario where D* Lite should show its advantage.
// =========================================================================

// --- D* Lite replan ---
static void BM_ReplanCompare_DStarLite(benchmark::State& state) {
    int size      = 300;
    int n_new_obs = static_cast<int>(state.range(0));

    for (auto _ : state) {
        state.PauseTiming();

        astar::GridMap grid(size, size);
        generateObstacles(grid, 20, static_cast<uint64_t>(state.iterations()));

        auto iw = [&grid](const astar::Vec3& c) { return grid.isWalkableCell(c.x, c.y); };
        auto gc = [&grid](const astar::Vec3& c) { return grid.getCostCell(c.x, c.y); };

        astar::DStarLiteAlgorithm engine(size, size, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.initialize({0, 0, 0}, {size - 1, size - 1, 0});
        engine.computeShortestPath();
        auto initial_path = engine.extractPath();
        if (initial_path.size() < 3) { state.SkipWithError("No init"); continue; }

        std::mt19937_64 rng(static_cast<uint64_t>(state.iterations()) + 12345);
        std::uniform_int_distribution<size_t> pick(initial_path.size() / 4,
                                                    initial_path.size() * 3 / 4);

        int added = 0;
        for (int i = 0; i < n_new_obs * 3 && added < n_new_obs; ++i) {
            auto& cell = initial_path[pick(rng)];
            engine.setObstacle(cell);
            ++added;
        }

        state.ResumeTiming();

        engine.replan({0, 0, 0});
        auto replanned = engine.extractPath();
        benchmark::DoNotOptimize(replanned);

        state.counters["init_len"] = static_cast<double>(initial_path.size());
        state.counters["replan_len"] = static_cast<double>(replanned.size());
        state.counters["obs_added"] = static_cast<double>(added);
    }
}
BENCHMARK(BM_ReplanCompare_DStarLite)
    ->ArgName("new_obs")
    ->DenseRange(1, 40, 10)
    ->Unit(benchmark::kMicrosecond);

// --- A* full replan ---
static void BM_ReplanCompare_AStar(benchmark::State& state) {
    int size      = 300;
    int n_new_obs = static_cast<int>(state.range(0));

    for (auto _ : state) {
        state.PauseTiming();

        astar::GridMap grid(size, size);
        generateObstacles(grid, 20, static_cast<uint64_t>(state.iterations()));

        auto iw = [&grid](const astar::Vec3& c) { return grid.isWalkableCell(c.x, c.y); };
        auto gc = [&grid](const astar::Vec3& c) { return grid.getCostCell(c.x, c.y); };

        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        algo.setGoalTolerance(0.0);

        auto initial_path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        if (!initial_path || initial_path->size() < 3) { state.SkipWithError("No init"); continue; }

        std::mt19937_64 rng(static_cast<uint64_t>(state.iterations()) + 12345);
        std::uniform_int_distribution<size_t> pick(initial_path->size() / 4,
                                                    initial_path->size() * 3 / 4);

        int added = 0;
        for (int i = 0; i < n_new_obs * 3 && added < n_new_obs; ++i) {
            auto& cell = (*initial_path)[pick(rng)];
            grid.setObstacle(cell.x, cell.y, true);
            ++added;
        }

        state.ResumeTiming();

        auto replanned = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(replanned);

        state.counters["init_len"] = static_cast<double>(initial_path->size());
        state.counters["replan_len"] = replanned.has_value()
            ? static_cast<double>(replanned->size()) : 0.0;
        state.counters["obs_added"] = static_cast<double>(added);
    }
}
BENCHMARK(BM_ReplanCompare_AStar)
    ->ArgName("new_obs")
    ->DenseRange(1, 40, 10)
    ->Unit(benchmark::kMicrosecond);

// =========================================================================
// 5. CrossCompare — side-by-side at 500², 10% obstacles
// =========================================================================

BENCHMARK_DEFINE_F(PlannerBench, BM_CrossCompare_AStar)(benchmark::State& state) {
    int size = 500;
    auto [iw, gc] = makeLambdas();
    for (auto _ : state) {
        state.PauseTiming();
        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        state.ResumeTiming();
        auto path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(path);
        state.counters["path_len"] = path.has_value()
            ? static_cast<double>(path->size()) : 0.0;
    }
}
BENCHMARK_REGISTER_F(PlannerBench, BM_CrossCompare_AStar)
    ->Args({500, 10})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(PlannerBench, BM_CrossCompare_DStarLite)(benchmark::State& state) {
    int size = 500;
    auto [iw, gc] = makeLambdas();
    for (auto _ : state) {
        state.PauseTiming();
        astar::DStarLiteAlgorithm engine(size, size, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.initialize({0, 0, 0}, {size - 1, size - 1, 0});
        state.ResumeTiming();
        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);
        state.counters["path_len"] = static_cast<double>(path.size());
    }
}
BENCHMARK_REGISTER_F(PlannerBench, BM_CrossCompare_DStarLite)
    ->Args({500, 10})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
