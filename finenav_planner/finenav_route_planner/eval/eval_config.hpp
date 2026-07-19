// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  Route Planner — 独立评测参数配置（无 ROS 依赖）
//
//  使用方法：
//    1. 修改下面的参数
//    2. colcon build --packages-select finenav_route_planner
//    3. ./build/finenav_route_planner/eval/route_planner_eval
//
//  输出：
//    /tmp/dstarlite_eval/paths.csv     — 路径坐标
//    /tmp/dstarlite_eval/stats.csv     — 耗时、长度、代价
//    /tmp/dstarlite_eval/_config.json  — 参数快照（plot.py 自动读取）
// ─────────────────────────────────────────────────────────────────────────────

namespace cfg {

// ── 1. 场景选择 ──────────────────────────────────────────────────────────────
//   0 = 依次运行全部三个场景
//   1 = 空地图对角线（A* vs D* Lite 基础对比）
//   2 = 墙缺口重规划（initial 穿缺口 → replan 封口绕行）
//   3 = 地形代价权重扫描（调 terrain_weight 看路径变化）
constexpr int kScenario = 0;

// ── 2. 算法开关 ──────────────────────────────────────────────────────────────
//   两个可以同时打开，做 side-by-side 对比
constexpr bool kRunAStar     = true;   // 是否运行 A*
constexpr bool kRunDStarLite = true;   // 是否运行 D* Lite

// ── 3. 网格与连通性 ──────────────────────────────────────────────────────────
//   kGridSize:      网格边长（kGridSize × kGridSize）
//   kConnectivity:  4 = 四邻域（只能上下左右），8 = 八邻域（含对角线）
//   kHeuristicAStar: A* 启发式  0=Euclidean 1=Diagonal 2=Manhattan 3=Zero
//   kHeuristicDSL:   D* Lite 启发式（同上，默认 Manhattan）
constexpr int kGridSize       = 100;
constexpr int kConnectivity   = 8;
constexpr int kHeuristicAStar = 0;   // Euclidean
constexpr int kHeuristicDSL   = 2;   // Manhattan

// ── 4. 代价权重 ──────────────────────────────────────────────────────────────
//   movement_weight: 移动步长代价，越大路径越倾向短步（固定 1.0）
//   greedy_weight:   启发式放大倍数，>1.0 搜索更快但路径可能次优
//   terrain_weight:  地形代价乘数。小值→忽略地形直穿；大值→绕开高代价区
//   goal_tolerance:  目标容差半径（格数），0=必须精确到达
constexpr double kMovementWeight = 1.0;
constexpr double kGreedyWeight   = 1.0;
constexpr double kTerrainWeight  = 1.0;
constexpr double kGoalTolerance  = 0.0;

// ── 5. 起点 / 终点（场景 1-3 共用）───────────────────────────────────────────
//   起点 (0,0)，终点 (99,99)，形成对角线路径
constexpr int kStartX = 0,   kStartY = 0;
constexpr int kGoalX  = 99,  kGoalY  = 99;

// ── 6. 场景二：墙 + 缺口 + 封口 ──────────────────────────────────────────────
//
//   地图布局（x=50 处一堵墙，中间开缺口）：
//
//        y=99 ┌──────────────┐
//             │              │
//        y=70 │  ██████████  │  ← 上半段墙（gap_y1 起）
//             │  ██ GAP ██  │     缺口 39 格高
//        y=30 │  ██████████  │  ← 下半段墙（0 到 gap_y0）
//             │              │
//        y=0  └──────────────┘
//             x=50
//
//   initial: 水平穿过缺口 (0,50)→(99,50)，len=100
//   replan:  缺口中间 y∈[45,55] 被 3 格宽横杆封住，路径绕行 y=44
//
//   调参：缩小 gap → 通过更难；扩大 closure → 绕行更远
constexpr int kWallX     = 50;   // 墙的 x 坐标
constexpr int kGapY0     = 30;   // 缺口下边界（下半段墙顶）
constexpr int kGapY1     = 70;   // 缺口上边界（上半段墙底）
constexpr int kClosureY0 = 45;   // 封口下边界
constexpr int kClosureY1 = 55;   // 封口上边界

// ── 7. 场景三：地形代价带权重扫描 ────────────────────────────────────────────
//
//   地图中央有一道矩形高代价区域（模拟泥泞/崎岖/禁行）：
//     x∈[35,45], y∈[10,89]，基础代价 = 10.0
//
//   对每个 terrain_weight 跑一次 D* Lite + A*：
//     小 weight（0.01）→ 忽略代价，直穿（路径短、代价高）
//     大 weight（5.0） → 绕开高代价区（路径长、代价低）
//
//   调参：改 kTerrainBandCost 调惩罚力度；改 kWeightSweep 增减扫描点
constexpr double kTerrainBandCost   = 10.0;
constexpr int    kBandX0 = 35, kBandX1 = 45;   // 高代价带 x 范围
constexpr int    kBandY0 = 10, kBandY1 = 89;   // 高代价带 y 范围

inline constexpr double kWeightSweep[] = {
    0.01, 0.05, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0, 2.0, 5.0
};
inline constexpr int kWeightSweepCount =
    static_cast<int>(sizeof(kWeightSweep) / sizeof(kWeightSweep[0]));

// ── 8. 输出目录 ──────────────────────────────────────────────────────────────
//   所有 CSV、_config.json 写到这里
constexpr const char* kOutputDir = "/tmp/dstarlite_eval";

}  // namespace cfg
