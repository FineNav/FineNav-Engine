// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

namespace cfg {

// ── 1. Scenario selection ───────────────────────────────────────────────────
//   0 = Run ALL three scenarios sequentially
//   1 = Empty grid diagonal  (A* vs D* Lite)
//   2 = Wall gap replan      (D* Lite incremental)
//   3 = Terrain band sweep   (terrain_weight tuning)
constexpr int kScenario = 0;

// ── 2. Algorithm selection ───────────────────────────────────────────────────
constexpr bool kRunAStar     = true;
constexpr bool kRunDStarLite = true;

// ── 3. Common grid parameters ────────────────────────────────────────────────
constexpr int kGridSize       = 100;
constexpr int kConnectivity   = 8;
constexpr int kHeuristicAStar = 0;     // Euclidean
constexpr int kHeuristicDSL   = 2;     // Manhattan

// ── 4. Weight parameters ─────────────────────────────────────────────────────
constexpr double kMovementWeight = 1.0;
constexpr double kGreedyWeight   = 1.0;
constexpr double kTerrainWeight  = 1.0;
constexpr double kGoalTolerance  = 0.0;

// ── 5. Start / goal ──────────────────────────────────────────────────────────
constexpr int kStartX = 0,   kStartY = 0;
constexpr int kGoalX  = 99,  kGoalY  = 99;

// ── 6. Scenario 2 — wall with gap, replan seals the gap ──────────────────────
constexpr int kWallX     = 50;
constexpr int kGapY0     = 30;
constexpr int kGapY1     = 70;
constexpr int kClosureY0 = 45;
constexpr int kClosureY1 = 55;

// ── 7. Scenario 3 — terrain band weight sweep ────────────────────────────────
constexpr double kTerrainBandCost   = 10.0;
constexpr int    kBandX0 = 35, kBandX1 = 45;
constexpr int    kBandY0 = 10, kBandY1 = 89;

inline constexpr double kWeightSweep[] = {
    0.01, 0.05, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0, 2.0, 5.0
};
inline constexpr int kWeightSweepCount =
    static_cast<int>(sizeof(kWeightSweep) / sizeof(kWeightSweep[0]));

// ── 8. Output ────────────────────────────────────────────────────────────────
constexpr const char* kOutputDir = "/tmp/dstarlite_eval";

}  // namespace cfg
