// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>

#include "finenav_route_planner/vec3.hpp"
#include "finenav_route_planner/grid_search_utils.hpp"   // Connectivity, getNeighbors, calculateStepCost
#include "finenav_route_planner/heuristic.hpp"            // Heuristic, calculateHeuristic
#include "finenav_route_planner/dstarlite_priority_key.hpp"   // Key, kInf

namespace astar {

// =============================================================
// DStarLiteAlgorithm
//
// Stateful D* Lite search engine for 2-D / 3-D grid maps.
//
// The engine maintains g-values, rhs-values, the priority
// queue, and the km offset across multiple planning calls.
// This is what makes replanning incremental — only the
// affected portion of the state is recomputed.
//
// Neighbourhood enumeration is delegated to the shared
// grid_search_utils.hpp (getNeighbors, calculateStepCost),
// eliminating the duplication present in the standalone
// DStarLite build.
//
// Usage:
//   DStarLiteAlgorithm engine(rows, cols, is_walkable, get_cost);
//   engine.initialize(start, goal);
//   engine.computeShortestPath();
//   auto path = engine.extractPath();
//
//   // ... obstacles change ...
//   engine.setObstacle(5, 5);
//   engine.replan(new_start);
//   auto new_path = engine.extractPath();
// =============================================================

class DStarLiteAlgorithm {
public:
    using IsWalkableFn = std::function<bool(const Vec3& p)>;
    using GetCostFn    = std::function<double(const Vec3& p)>;

    // ── construction ──────────────────────────────────────────

    DStarLiteAlgorithm(int rows, int cols,
                   IsWalkableFn is_walkable,
                   GetCostFn    get_cost,
                   Connectivity connectivity = Connectivity::N8,
                   Heuristic    heuristic    = Heuristic::Diagonal,
                   int          layers       = 1);

    // ── grid queries (forward to callbacks) ───────────────────

    bool   isWalkable(const Vec3& p) const;
    double getCost(const Vec3& p)    const;

    // ── dimensions ────────────────────────────────────────────

    int rows()   const noexcept { return rows_; }
    int cols()   const noexcept { return cols_; }
    int layers() const noexcept { return layers_; }

    // ── connectivity / heuristic (hot-swappable — forces rebuild) ─

    void setConnectivity(Connectivity c);
    void setHeuristic(Heuristic h);

    Connectivity connectivity()  const noexcept { return connectivity_; }
    Heuristic    heuristicType() const noexcept { return heuristic_type_; }

    // ── obstacle modification (must call replan afterwards) ───

    void setObstacle(const Vec3& p);
    void removeObstacle(const Vec3& p);

    // ── map synchronisation ───────────────────────────────────

    /// Re-read the isWalkable callback for every cell and update
    /// the internal obstacle grid.  Cells that changed from free
    /// to blocked (or vice-versa) have their neighbours updated
    /// so they enter the priority queue.
    ///
    /// Call this before replan() when the external map has changed.
    void syncMap();

    /// Same as syncMap() but only scans cells within a Chebyshev
    /// distance of `radius_cells` from `around`.  This is much
    /// cheaper than a full-grid scan when the map is large and
    /// changes are local.
    void syncMapAround(const Vec3& around, int radius_cells);

    // ── algorithm lifecycle ───────────────────────────────────

    /// (Re)initialise all internal state.  Call once at the very
    /// beginning, before the first computeShortestPath().
    void initialize(const Vec3& start, const Vec3& goal);

    /// Main search loop — expands nodes until the start is
    /// consistent and optimal.  Call after initialize() and
    /// after every replan().
    void computeShortestPath();

    /// Replan after obstacle changes.  Updates km and re-runs
    /// computeShortestPath().
    void replan(const Vec3& new_start);

    /// Move the robot one step towards the goal.
    /// Updates the internal start position (does NOT update km).
    /// Returns false if stuck.
    bool step();

    // ── path extraction ──────────────────────────────────────

    /// Extract the current best path by following g-values from
    /// start to goal.  Returns an empty vector if unreachable.
    std::vector<Vec3> extractPath() const;

    // ── accessors ─────────────────────────────────────────────

    const Vec3& start() const noexcept { return start_; }
    const Vec3& goal()  const noexcept { return goal_; }

    double g(const Vec3& p)  const;
    double rhs(const Vec3& p) const;
    double km()              const noexcept { return km_; }

    // ── configurable limits ──────────────────────────────────

    void setMaxExpansions(int n)    noexcept { max_expansions_    = n; }
    void setMaxPathSteps(int n)     noexcept { max_path_steps_    = n; }
    void setMaxPlanningTime(double s) noexcept { max_planning_time_ = s; }
    void setGoalTolerance(int n)   noexcept { goal_tolerance_    = n; }

    int    maxExpansions()    const noexcept { return max_expansions_; }
    int    maxPathSteps()     const noexcept { return max_path_steps_; }
    double maxPlanningTime()  const noexcept { return max_planning_time_; }
    int    goalTolerance()    const noexcept { return goal_tolerance_; }

    // ── debug ─────────────────────────────────────────────────

    void printGrid(const std::vector<Vec3>& path = {}) const;

private:
    // ── core D* Lite functions ────────────────────────────────

    Key  calculateKey(const Vec3& s) const;
    void updateVertex(const Vec3& u);
    void insertOrUpdate(const Vec3& s);

    // ── neighbours & costs (delegated to grid_search_utils) ───

    std::vector<Vec3> getSuccessors(const Vec3& u)   const;
    std::vector<Vec3> getPredecessors(const Vec3& u) const;
    std::vector<Vec3> getAllNeighbors(const Vec3& u) const;
    double            moveCost(const Vec3& a, const Vec3& b) const;

    // ── helpers ───────────────────────────────────────────────

    int  idx(const Vec3& p) const noexcept {
        return (p.z * rows_ + p.y) * cols_ + p.x;
    }
    void checkBounds(const Vec3& p) const;

    // ── injected callbacks ────────────────────────────────────

    IsWalkableFn is_walkable_;
    GetCostFn    get_cost_;

    // ── grid ──────────────────────────────────────────────────

    int rows_;
    int cols_;
    int layers_;
    Connectivity   connectivity_;
    Heuristic      heuristic_type_;

    // ── D* Lite state ─────────────────────────────────────────

    std::vector<double> g_;
    std::vector<double> rhs_;
    std::vector<bool>   obstacles_;   // tracked internally for efficiency
    Vec3 start_{0, 0, 0};
    Vec3 goal_{0, 0, 0};
    double km_    = 0.0;
    bool   valid_ = false;   // true after first initialize()

    // ── configurable limits ─────────────────────────────────

    int    max_expansions_    = 1'000'000;   // -1 = unlimited
    int    max_path_steps_    = 10'000;      // -1 = unlimited
    double max_planning_time_ = 5.0;         // 0.0 = unlimited
    int    goal_tolerance_    = 0;           // 0 = exact goal

    // ── priority queue ────────────────────────────────────────

    using PQEntry = std::pair<Key, Vec3>;
    struct PQCompare {
        bool operator()(const PQEntry& a, const PQEntry& b) const noexcept {
            return a.first > b.first;  // min-heap
        }
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, PQCompare> U_;
    std::unordered_map<Vec3, Key, Vec3Hash> key_cache_;

    // ── thread safety ────────────────────────────────────────

    mutable std::recursive_mutex mutex_;
};

}  // namespace astar
