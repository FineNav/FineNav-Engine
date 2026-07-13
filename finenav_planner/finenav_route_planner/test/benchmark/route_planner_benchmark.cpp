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

// =========================================================================
// 6. Weight Sensitivity — how movement_weight and greedy_weight affect paths
//
//    Grid: 100×100, no obstacles but a high-cost terrain band (cost=5.0)
//    across the middle.  The planner must choose: cut through expensive
//    terrain (shorter) or go around (longer but cheaper).
//
//    This is the exact scenario the user tunes weights for:
//      - movement_weight  controls distance vs terrain-cost trade-off
//      - greedy_weight    controls search greediness (may affect optimality)
// =========================================================================

// --- 6a. movement_weight sweep ---

static void BM_Weight_MovementWeight(benchmark::State& state) {
    const int W = 100, H = 100;
    double mw = state.range(0) * 0.1;  // mw_x10 → movement_weight

    for (auto _ : state) {
        state.PauseTiming();

        // Build grid with a high-cost band (x ∈ [35,45], y ∈ [10,89])
        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 5.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        astar::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(mw);
        engine.setGreedyWeight(1.0);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        // Count cells in the high-cost band
        int swamp_cells = 0;
        double total_terrain_cost = 0.0;
        for (const auto& c : path) {
            total_terrain_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_terrain_cost;
    }
}
BENCHMARK(BM_Weight_MovementWeight)
    ->ArgName("mw_x10")
    ->Arg(1)      // mw=0.1  terrain-sensitive — avoid high-cost at all costs
    ->Arg(5)      // mw=0.5  slightly terrain-averse
    ->Arg(10)     // mw=1.0  default
    ->Arg(50)     // mw=5.0  distance-preferring
    ->Arg(100)    // mw=10.0 distance-first — aggressively cut through
    ->Unit(benchmark::kMillisecond);

// --- 6b. greedy_weight sweep ---

static void BM_Weight_GreedyWeight(benchmark::State& state) {
    const int W = 100, H = 100;
    double gw = state.range(0) * 0.1;  // gw_x10 → greedy_weight

    for (auto _ : state) {
        state.PauseTiming();

        // Same high-cost band setup
        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 5.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        astar::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(1.0);
        engine.setGreedyWeight(gw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_GreedyWeight)
    ->ArgName("gw_x10")
    ->Arg(5)     // gw=0.5  deflated heuristic — Dijkstra-like, slow but safe
    ->Arg(10)    // gw=1.0  default optimal
    ->Arg(15)    // gw=1.5  slightly greedy — bounded suboptimal
    ->Arg(20)    // gw=2.0  more greedy — may cut corners
    ->Arg(50)    // gw=5.0  very greedy — aggressive focus toward start
    ->Unit(benchmark::kMillisecond);

// --- 6c. combined: movement_weight × greedy_weight at critical points ---

static void BM_Weight_Combined(benchmark::State& state) {
    const int W = 100, H = 100;
    double mw = state.range(0) * 0.1;   // mw_x10 → actual movement_weight
    double gw = state.range(1);          // greedy_weight (integer)

    for (auto _ : state) {
        state.PauseTiming();

        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 5.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        astar::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(mw);
        engine.setGreedyWeight(gw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_Combined)
    // (mw_x10, gw) pairs → actual mw = mw_x10/10
    ->Args({1, 1})      // mw=0.1 terrain-sensitive,  gw=1.0 optimal
    ->Args({10, 1})     // mw=1.0 default,             gw=1.0 optimal
    ->Args({100, 1})    // mw=10.0 distance-first,      gw=1.0 optimal
    ->Args({1, 2})      // mw=0.1 terrain-sensitive,  gw=2.0 faster
    ->Args({10, 2})     // mw=1.0 default,             gw=2.0 faster
    ->Args({100, 2})    // mw=10.0 distance-first,      gw=2.0 faster
    ->Args({1, 5})      // mw=0.1 terrain-sensitive,  gw=5.0 aggressive
    ->Args({10, 5})     // mw=1.0 default,             gw=5.0 aggressive
    ->Args({100, 5})    // mw=10.0 distance-first,      gw=5.0 aggressive
    ->ArgNames({"mw_x10", "gw"})
    ->Unit(benchmark::kMillisecond);

// --- 6d. terrain_weight sweep — how map cost scaling changes path choice ---

static void BM_Weight_TerrainWeight(benchmark::State& state) {
    const int W = 100, H = 100;
    double tw = state.range(0) * 0.01;  // tw_x100 → terrain_weight

    for (auto _ : state) {
        state.PauseTiming();

        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 10.0;  // moderate raw cost so tw transition is visible

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        astar::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(1.0);
        engine.setGreedyWeight(1.0);
        engine.setTerrainWeight(tw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_TerrainWeight)
    ->ArgName("tw_x100")
    ->Arg(1)      // tw=0.01   terrain nearly ignored → straight through
    ->Arg(5)      // tw=0.05   terrain barely matters
    ->Arg(10)     // tw=0.1    transition zone
    ->Arg(50)     // tw=0.5    somewhat terrain-averse
    ->Arg(100)    // tw=1.0    default — should go around
    ->Arg(500)    // tw=5.0    terrain dominates
    ->Unit(benchmark::kMillisecond);

// --- 6e. movement_weight × terrain_weight — the two "distance vs map" knobs together ---

static void BM_Weight_MWxTW(benchmark::State& state) {
    const int W = 100, H = 100;
    double mw = state.range(0) * 0.1;   // mw_x10 → movement_weight
    double tw = state.range(1) * 0.01;  // tw_x100 → terrain_weight

    for (auto _ : state) {
        state.PauseTiming();

        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 500.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        astar::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(mw);
        engine.setGreedyWeight(1.0);
        engine.setTerrainWeight(tw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_MWxTW)
    ->Args({1, 1})       // mw=0.1  tw=0.01  distance-cheap, terrain-ignored
    ->Args({1, 1000})    // mw=0.1  tw=10.0  distance-cheap, terrain-heavy
    ->Args({100, 1})     // mw=10.0 tw=0.01  distance-heavy, terrain-ignored
    ->Args({100, 1000})  // mw=10.0 tw=10.0  both heavy — balanced competition
    ->Args({10, 100})    // mw=1.0  tw=1.0   default
    ->ArgNames({"mw_x10", "tw_x100"})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
