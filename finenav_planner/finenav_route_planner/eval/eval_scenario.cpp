// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file eval_scenario.cpp
 * @brief Route Planner 独立可视化评估 — 多场景仿真 + CSV 输出
 *
 * @details
 * ==========================================================================
 * 总览
 * ==========================================================================
 * 本文件是 D* Lite / A* 算法的 standalone 可视化测试工具，
 * 完全无 ROS 依赖，仅依赖算法核心文件和 GridMap 测试工具。
 *
 * 设计模式参考 finenav_localizer/eval：
 *   eval_config.hpp  →  编译期参数配置
 *   eval_scenario.cpp →  场景模拟 + CSV 输出
 *   plot.py          →  matplotlib 可视化
 *
 * 三个场景:
 *   1 = 空地图对角线 (A* vs D* Lite 基础对比)
 *   2 = 障碍墙 replan (D* Lite 增量优势展示)
 *   3 = 地形带权重扫描 (terrain_weight 调参工具)
 *
 * ==========================================================================
 * 构建与运行
 * ==========================================================================
 * 构建:
 * @code
 *   cd ~/FineNav_ws
 *   colcon build --symlink-install --packages-select finenav_route_planner
 * @endcode
 *
 * 运行 (每次改 eval_config.hpp 的 kScenario 后重新编译):
 * @code
 *   ./build/finenav_route_planner/eval/route_planner_eval
 * @endcode
 *
 * 可视化:
 * @code
 *   python3 finenav_planner/finenav_route_planner/eval/plot.py /tmp/dstarlite_eval
 * @endcode
 *
 * ==========================================================================
 * CSV 输出格式
 * ==========================================================================
 * paths.csv:
 *   algo,scenario,run,step,x,y,label
 *
 * stats.csv:
 *   algo,scenario,run,path_type,length,compute_ms,total_terrain_cost
 *
 * @author Jinghui Wang (wangjinghui@sjtu.edu.cn)
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-07-19
 */

#include "eval_config.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "common/grid_map.hpp"
#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/dstarlite/dstarlite_algorithm.hpp"

namespace fs = std::filesystem;

// ===== 公共类型 =====

/// 一次规划运行的结果
struct PathResult {
    std::string algo;            ///< "AStar" | "DStarLite"
    std::string scenario;        ///< 场景名 (CSV 用)
    std::string label;           ///< "initial" | "replan" | "tw=0.01" ...
    int         run = 0;         ///< 同场景内多次运行的序号
    std::vector<astar::Vec3> path;
    double      compute_ms   = 0.0;
    double      total_terrain_cost = 0.0;
};

/// 一次 eval 运行的所有结果
struct EvalRun {
    std::string              name;
    std::vector<PathResult>  paths;
};

// ===== 辅助函数 =====

/// 创建空网格 (所有 cell 可行走, 代价 0)
static astar::GridMap buildEmptyGrid(int size) {
    return astar::GridMap(size, size, 0.0);
}

/// 创建带缺口的两段墙（Scenario 2）
/// x=wall_x, 下半段 y∈[0, gap_y0], 上半段 y∈[gap_y1, size-1],
/// 中间缺口 y∈(gap_y0, gap_y1) 可通行。
static astar::GridMap buildGapWallGrid(int size, int wall_x,
                                        int gap_y0, int gap_y1) {
    astar::GridMap grid(size, size, 0.0);
    for (int y = 0; y <= gap_y0; ++y)
        grid.setObstacle(wall_x, y, true);
    for (int y = gap_y1; y < size; ++y)
        grid.setObstacle(wall_x, y, true);
    return grid;
}

/// 创建带矩形高代价地形带的网格
/// @param x0,x1,y0,y1 地形带范围
/// @param cost 地形带内代价
static astar::GridMap buildTerrainBandGrid(int size,
                                            int x0, int x1, int y0, int y1,
                                            double cost) {
    astar::GridMap grid(size, size, 0.0);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
            grid.setCost(x, y, cost);
    return grid;
}

/// 从 GridMap 构建 lambda 回调对
static auto makeLambdas(const astar::GridMap& g) {
    return std::make_pair(
        [&g](const astar::Vec3& c) { return g.isWalkableCell(c.x, c.y); },
        [&g](const astar::Vec3& c) { return g.getCostCell(c.x, c.y); }
    );
}

/// Heuristic 枚举 → astar::Heuristic
static astar::Heuristic toHeuristic(int h) {
    switch (h) {
        case 0:  return astar::Heuristic::Euclidean;
        case 1:  return astar::Heuristic::Diagonal;
        case 2:  return astar::Heuristic::Manhattan;
        case 3:  return astar::Heuristic::Zero;
        default: return astar::Heuristic::Euclidean;
    }
}

/// Connectivity int → astar::Connectivity
static astar::Connectivity toConn(int c) {
    return (c == 4) ? astar::Connectivity::N4 : astar::Connectivity::N8;
}

/// 计算路径上地形代价总和
static double terrainCostSum(const std::vector<astar::Vec3>& path,
                              const astar::GridMap& grid) {
    double sum = 0.0;
    for (const auto& c : path)
        sum += grid.getCostCell(c.x, c.y);
    return sum;
}

/// 确保输出目录存在
static void ensureOutputDir() {
    std::error_code ec;
    fs::create_directories(cfg::kOutputDir, ec);
}

// ===== CSV 输出 =====

/// 写入 paths.csv
static void writePathsCSV(const std::vector<PathResult>& results) {
    ensureOutputDir();
    std::string p = std::string(cfg::kOutputDir) + "/paths.csv";
    std::ofstream f(p);
    if (!f) {
        std::cerr << "[ERROR] Cannot write " << p << "\n";
        return;
    }
    f << "algo,scenario,run,step,x,y,label\n";
    for (const auto& r : results) {
        for (size_t i = 0; i < r.path.size(); ++i) {
            f << r.algo << ","
              << r.scenario << ","
              << r.run << ","
              << i << ","
              << r.path[i].x << ","
              << r.path[i].y << ","
              << r.label << "\n";
        }
    }
    std::cout << "[paths.csv] " << results.size() << " paths → " << p << "\n";
}

/// 写入 stats.csv
static void writeStatsCSV(const std::vector<PathResult>& results) {
    ensureOutputDir();
    std::string p = std::string(cfg::kOutputDir) + "/stats.csv";
    std::ofstream f(p);
    if (!f) {
        std::cerr << "[ERROR] Cannot write " << p << "\n";
        return;
    }
    f << "algo,scenario,run,path_type,length,compute_ms,total_terrain_cost\n";
    for (const auto& r : results) {
        f << r.algo << ","
          << r.scenario << ","
          << r.run << ","
          << r.label << ","
          << r.path.size() << ","
          << r.compute_ms << ","
          << r.total_terrain_cost << "\n";
    }
    std::cout << "[stats.csv] " << results.size() << " rows → " << p << "\n";
}

// ===== 高精度计时 =====

using Clock = std::chrono::high_resolution_clock;

static double elapsedMs(Clock::time_point t0, Clock::time_point t1) {
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// =========================================================================
// Scenario 1 — 空地图对角线 (A* vs D* Lite 对比)
//
// 100×100 空网格, (0,0)→(99,99) 对角线路径。
// 同时跑 A* (Euclidean) 和 D* Lite (Diagonal)，输出两条路径供对比。
// 预期: 两者找到相同的最短路径 (或长度相同)。
// =========================================================================
static EvalRun runScenario1() {
    EvalRun run;
    run.name = "empty_diagonal";

    const int sz = cfg::kGridSize;
    astar::GridMap grid = buildEmptyGrid(sz);
    auto [iw, gc] = makeLambdas(grid);

    std::vector<PathResult> results;

    // ── A* ──
    if constexpr (cfg::kRunAStar) {
        PathResult r;
        r.algo     = "AStar";
        r.scenario = "empty_diagonal";
        r.label    = "initial";

        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(toConn(cfg::kConnectivity));
        algo.setHeuristic(toHeuristic(cfg::kHeuristicAStar));
        algo.setGoalTolerance(cfg::kGoalTolerance);

        auto t0 = Clock::now();
        auto opt = algo.findPath({cfg::kStartX, cfg::kStartY, 0},
                                  {cfg::kGoalX,  cfg::kGoalY,  0});
        auto t1 = Clock::now();
        r.compute_ms = elapsedMs(t0, t1);

        if (opt) {
            r.path = *opt;
            r.total_terrain_cost = terrainCostSum(r.path, grid);
        }
        results.push_back(r);
    }

    // ── D* Lite ──
    if constexpr (cfg::kRunDStarLite) {
        PathResult r;
        r.algo     = "DStarLite";
        r.scenario = "empty_diagonal";
        r.label    = "initial";

        astar::DStarLiteAlgorithm engine(sz, sz, iw, gc,
                                          toConn(cfg::kConnectivity),
                                          toHeuristic(cfg::kHeuristicDSL));
        engine.setMovementWeight(cfg::kMovementWeight);
        engine.setGreedyWeight(cfg::kGreedyWeight);
        engine.setTerrainWeight(cfg::kTerrainWeight);

        engine.initialize({cfg::kStartX, cfg::kStartY, 0},
                           {cfg::kGoalX,  cfg::kGoalY,  0});

        auto t0 = Clock::now();
        engine.computeShortestPath();
        auto t1 = Clock::now();
        r.compute_ms = elapsedMs(t0, t1);

        r.path = engine.extractPath();
        r.total_terrain_cost = terrainCostSum(r.path, grid);
        results.push_back(r);
    }

    run.paths = results;
    return run;
}

// =========================================================================
// Scenario 2 — 墙中间有缺口，replan 封口绕行
//
// x=50 处有两段墙：下半段 y∈[0, kGapY0]，上半段 y∈[kGapY1, 99]，
// 中间缺口 y∈(kGapY0, kGapY1) 是唯一通道。
// 初始路径从 (0,50) 水平穿过缺口到达 (99,50)。
// Replan 在缺口中间封一道横杆，迫使绕行上方或下方的剩余缺口。
// =========================================================================
static EvalRun runScenario2() {
    EvalRun run;
    run.name = "wall_replan";

    const int sz = cfg::kGridSize;
    const int sx = 0,  sy = 50;
    const int gx = 99, gy = 50;
    std::vector<PathResult> results;

    auto run_dstar = [&](bool closure) -> std::vector<PathResult> {
        std::vector<PathResult> out;
        astar::GridMap grid = buildGapWallGrid(
            sz, cfg::kWallX, cfg::kGapY0, cfg::kGapY1);
        auto [iw, gc] = makeLambdas(grid);

        astar::DStarLiteAlgorithm engine(sz, sz, iw, gc,
                                          toConn(cfg::kConnectivity),
                                          toHeuristic(cfg::kHeuristicDSL));
        engine.setMovementWeight(cfg::kMovementWeight);
        engine.setGreedyWeight(cfg::kGreedyWeight);
        engine.setTerrainWeight(cfg::kTerrainWeight);

        engine.initialize({sx, sy, 0}, {gx, gy, 0});

        auto t0 = Clock::now();
        engine.computeShortestPath();
        auto t1 = Clock::now();

        PathResult init;
        init.algo     = "DStarLite";
        init.scenario = "wall_replan";
        init.label    = "initial";
        init.path     = engine.extractPath();
        init.compute_ms = elapsedMs(t0, t1);
        init.total_terrain_cost = terrainCostSum(init.path, grid);
        out.push_back(init);

        if (closure && !init.path.empty()) {
            // 封口：在缺口中间封一道横杆
            for (int y = cfg::kClosureY0; y <= cfg::kClosureY1; ++y)
                for (int x = cfg::kWallX - 1; x <= cfg::kWallX + 1; ++x)
                    grid.setObstacle(x, y, true);

            engine.syncMap();

            // 从起点 replan——机器人尚未移动即检测到封口变化。
            // 此时 g_start 足够大，欠一致节点的 key < start_key，
            // 标准 D* Lite 终止条件自然处理它们。
            t0 = Clock::now();
            engine.replan({sx, sy, 0});
            t1 = Clock::now();

            PathResult repl;
            repl.algo     = "DStarLite";
            repl.scenario = "wall_replan";
            repl.label    = "replan";
            repl.path     = engine.extractPath();
            repl.compute_ms = elapsedMs(t0, t1);
            repl.total_terrain_cost = terrainCostSum(repl.path, grid);
            out.push_back(repl);
        }
        return out;
    };

    auto run_astar = [&](bool closure) -> std::vector<PathResult> {
        std::vector<PathResult> out;
        astar::GridMap grid = buildGapWallGrid(
            sz, cfg::kWallX, cfg::kGapY0, cfg::kGapY1);
        auto [iw, gc] = makeLambdas(grid);

        PathResult init;
        init.algo     = "AStar";
        init.scenario = "wall_replan";
        init.label    = "initial";

        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(toConn(cfg::kConnectivity));
        algo.setHeuristic(toHeuristic(cfg::kHeuristicAStar));
        algo.setGoalTolerance(cfg::kGoalTolerance);

        auto t0 = Clock::now();
        auto opt = algo.findPath({sx, sy, 0}, {gx, gy, 0});
        auto t1 = Clock::now();
        init.compute_ms = elapsedMs(t0, t1);
        if (opt) {
            init.path = *opt;
            init.total_terrain_cost = terrainCostSum(init.path, grid);
        }
        out.push_back(init);

        if (closure && opt && opt->size() >= 3) {
            for (int y = cfg::kClosureY0; y <= cfg::kClosureY1; ++y)
                for (int x = cfg::kWallX - 1; x <= cfg::kWallX + 1; ++x)
                    grid.setObstacle(x, y, true);

            astar::AStarAlgorithm algo2(iw, gc);
            algo2.setConnectivity(toConn(cfg::kConnectivity));
            algo2.setHeuristic(toHeuristic(cfg::kHeuristicAStar));
            algo2.setGoalTolerance(cfg::kGoalTolerance);

            t0 = Clock::now();
            auto opt2 = algo2.findPath({sx, sy, 0}, {gx, gy, 0});
            t1 = Clock::now();

            PathResult repl;
            repl.algo     = "AStar";
            repl.scenario = "wall_replan";
            repl.label    = "replan";
            repl.compute_ms = elapsedMs(t0, t1);
            if (opt2) {
                repl.path = *opt2;
                repl.total_terrain_cost = terrainCostSum(repl.path, grid);
            }
            out.push_back(repl);
        }
        return out;
    };

    if constexpr (cfg::kRunDStarLite) {
        auto v = run_dstar(true);
        results.insert(results.end(), v.begin(), v.end());
    }
    if constexpr (cfg::kRunAStar) {
        auto v = run_astar(true);
        results.insert(results.end(), v.begin(), v.end());
    }

    run.paths = results;
    return run;
}

// =========================================================================
// Scenario 3 — 地形带权重扫描
//
// 100×100 网格, 中央高代价 band (x∈[35,45], y∈[10,89], cost=10.0)。
// 循环 kWeightSweep 中每个 terrain_weight,
// 每次跑一次 D* Lite computeShortestPath。
// 观察 terrain_weight 如何影响路径选择 (穿过 vs 绕路)。
// =========================================================================
static EvalRun runScenario3() {
    EvalRun run;
    run.name = "terrain_sweep";

    const int sz = cfg::kGridSize;
    astar::GridMap grid = buildTerrainBandGrid(
        sz, cfg::kBandX0, cfg::kBandX1,
        cfg::kBandY0, cfg::kBandY1, cfg::kTerrainBandCost);
    auto [iw, gc] = makeLambdas(grid);

    std::vector<PathResult> results;

    for (int wi = 0; wi < cfg::kWeightSweepCount; ++wi) {
        double tw = cfg::kWeightSweep[wi];

        // ── D* Lite ──
        if constexpr (cfg::kRunDStarLite) {
            PathResult r;
            r.algo     = "DStarLite";
            r.scenario = "terrain_sweep";
            r.run      = wi;

            // Label with weight value for plot legend
            char buf[32];
            std::snprintf(buf, sizeof(buf), "tw=%.2f", tw);
            r.label = buf;

            astar::DStarLiteAlgorithm engine(sz, sz, iw, gc,
                                              toConn(cfg::kConnectivity),
                                              toHeuristic(cfg::kHeuristicDSL));
            engine.setMovementWeight(cfg::kMovementWeight);
            engine.setGreedyWeight(cfg::kGreedyWeight);
            engine.setTerrainWeight(tw);

            engine.initialize({cfg::kStartX, cfg::kGoalX == 99 ? 50 : cfg::kStartY, 0},
                               {cfg::kGoalX,  cfg::kGoalX == 99 ? 50 : cfg::kGoalY,  0});

            auto t0 = Clock::now();
            engine.computeShortestPath();
            auto t1 = Clock::now();
            r.compute_ms = elapsedMs(t0, t1);

            r.path = engine.extractPath();
            r.total_terrain_cost = terrainCostSum(r.path, grid);
            results.push_back(r);
        }

        // ── A* (same weights, for comparison) ──
        if constexpr (cfg::kRunAStar) {
            PathResult r;
            r.algo     = "AStar";
            r.scenario = "terrain_sweep";
            r.run      = wi;

            char buf[32];
            std::snprintf(buf, sizeof(buf), "tw=%.2f", tw);
            r.label = buf;

            astar::AStarAlgorithm algo(iw, gc);
            algo.setConnectivity(toConn(cfg::kConnectivity));
            algo.setHeuristic(toHeuristic(cfg::kHeuristicAStar));
            algo.setGoalTolerance(cfg::kGoalTolerance);
            algo.setMovementWeight(cfg::kMovementWeight);
            algo.setGreedyWeight(cfg::kGreedyWeight);
            algo.setTerrainWeight(tw);

            auto t0 = Clock::now();
            auto opt = algo.findPath({cfg::kStartX, cfg::kGoalX == 99 ? 50 : cfg::kStartY, 0},
                                      {cfg::kGoalX,  cfg::kGoalX == 99 ? 50 : cfg::kGoalY,  0});
            auto t1 = Clock::now();
            r.compute_ms = elapsedMs(t0, t1);

            if (opt) {
                r.path = *opt;
                r.total_terrain_cost = terrainCostSum(r.path, grid);
            }
            results.push_back(r);
        }
    }

    run.paths = results;
    return run;
}

// ===== main =====

/// RAII 计时器 — 打印阶段耗时
struct StageTimer {
    const char* name;
    Clock::time_point t0;
    explicit StageTimer(const char* n) : name(n), t0(Clock::now()) {}
    ~StageTimer() {
        auto t1 = Clock::now();
        std::cout << "[" << name << "] done in " << elapsedMs(t0, t1) << " ms\n";
    }
};

/// 写 _config.json 供 plot.py 自动同步参数
static void writeConfigJSON() {
    ensureOutputDir();
    std::string p = std::string(cfg::kOutputDir) + "/_config.json";
    std::ofstream f(p);
    if (!f) return;
    f << "{\n"
      << "  \"grid_size\": " << cfg::kGridSize << ",\n"
      << "  \"wall_x\": " << cfg::kWallX << ",\n"
      << "  \"gap_y0\": " << cfg::kGapY0 << ",\n"
      << "  \"gap_y1\": " << cfg::kGapY1 << ",\n"
      << "  \"closure_y0\": " << cfg::kClosureY0 << ",\n"
      << "  \"closure_y1\": " << cfg::kClosureY1 << ",\n"
      << "  \"band_x0\": " << cfg::kBandX0 << ",\n"
      << "  \"band_x1\": " << cfg::kBandX1 << ",\n"
      << "  \"band_y0\": " << cfg::kBandY0 << ",\n"
      << "  \"band_y1\": " << cfg::kBandY1 << ",\n"
      << "  \"band_cost\": " << cfg::kTerrainBandCost << "\n"
      << "}\n";
    std::cout << "[_config.json] → " << p << "\n";
}

/// 打印格式化的数据对比表格
static void printResultTable(const std::vector<PathResult>& results) {
    if (results.empty()) return;

    // 列宽
    const int w_algo = 10, w_label = 10, w_len = 8, w_time = 10, w_cost = 8;
    auto sep = [&](const char* l, const char* m, const char* r) {
        std::cout << l;
        for (int i = 0; i < w_algo + w_label + w_len + w_time + w_cost + 14; ++i)
            std::cout << "─";
        std::cout << r << "\n";
    };

    sep("┌", "┬", "┐");
    printf("│ %-*s │ %-*s │ %*s │ %*s │ %*s │\n",
           w_algo, "Algorithm", w_label, "Stage",
           w_len, "Length", w_time, "Time(ms)", w_cost, "Cost");
    sep("├", "┼", "┤");

    for (const auto& r : results) {
        printf("│ %-*s │ %-*s │ %*d │ %*.2f │ %*.0f │\n",
               w_algo, r.algo.c_str(),
               w_label, r.label.c_str(),
               w_len,   (int)r.path.size(),
               w_time,  r.compute_ms,
               w_cost,  r.total_terrain_cost);
    }
    sep("└", "┴", "┘");
    std::cout << "\n";
}

/// 处理单个 scenario 的 CSV 输出 + 自动调用 plot.py
static void finishScenario(const EvalRun& run) {
    writePathsCSV(run.paths);
    writeStatsCSV(run.paths);
    writeConfigJSON();

    std::cout << "\n── " << run.name << " ──\n";
    printResultTable(run.paths);

    // Auto-invoke plot.py（路径由 CMake 编译时注入，适配任意克隆位置）
    std::string cmd = std::string("python3 ") + EVAL_DIR + "/plot.py "
                    + std::string(cfg::kOutputDir);
    std::cout << "\n[plot] " << std::flush;
    int ret = std::system(cmd.c_str());
    if (ret != 0)
        std::cerr << "[plot] Warning: plot.py exited with code " << ret << "\n";

    // 清理临时文件
    std::error_code ec;
    fs::remove(std::string(cfg::kOutputDir) + "/paths.csv", ec);
    fs::remove(std::string(cfg::kOutputDir) + "/stats.csv", ec);
    fs::remove(std::string(cfg::kOutputDir) + "/_config.json", ec);
    std::cout << "\n";
}

int main() {
    std::cout << "═══════════════════════════════════════════\n";
    std::cout << " Route Planner — Standalone Eval\n";
    std::cout << " Scenario: " << (cfg::kScenario == 0 ? "ALL (1+2+3)" :
                                  cfg::kScenario == 1 ? "1 — empty_diagonal" :
                                  cfg::kScenario == 2 ? "2 — wall_replan" :
                                  "3 — terrain_sweep") << "\n";
    std::cout << " Grid:     " << cfg::kGridSize << "×" << cfg::kGridSize << "\n";
    std::cout << " A*:       " << (cfg::kRunAStar     ? "ON" : "OFF") << "\n";
    std::cout << " D* Lite:  " << (cfg::kRunDStarLite ? "ON" : "OFF") << "\n";
    std::cout << " Output:   " << cfg::kOutputDir << "\n";
    std::cout << "═══════════════════════════════════════════\n\n";

    if constexpr (cfg::kScenario == 0) {
        // ── Run all three scenarios ──
        {
            StageTimer st("Scenario 1 — empty_diagonal");
            finishScenario(runScenario1());
        }
        {
            StageTimer st("Scenario 2 — wall_replan");
            finishScenario(runScenario2());
        }
        {
            StageTimer st("Scenario 3 — terrain_sweep");
            finishScenario(runScenario3());
        }
        std::cout << "\n[Done] All 3 scenarios complete.  PNGs saved in "
                  << cfg::kOutputDir << "\n";
    } else {
        // ── Single scenario ──
        EvalRun run;
        switch (cfg::kScenario) {
            case 1: run = runScenario1(); break;
            case 2: run = runScenario2(); break;
            case 3: run = runScenario3(); break;
            default:
                std::cerr << "[ERROR] Unknown cfg::kScenario = "
                          << cfg::kScenario << " (valid: 0, 1, 2, 3)\n";
                return 1;
        }
        finishScenario(run);
    }

    return 0;
}
