// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "bfs_vec3.hpp"

namespace bfs {

// =============================================================
// BfsAlgorithm
//
// A plain uniform-grid breadth-first search engine.
//
// It ignores traversal cost and expands nodes level by level, so
// the returned path has the fewest grid hops under the configured
// connectivity.
// =============================================================

class BfsAlgorithm {
public:
    enum class Connectivity { N4, N8, N6, N18, N26 };

    using IsWalkableFn = std::function<bool(const Vec3&)>;
    using IsGoalFn = std::function<bool(const Vec3&)>;

    explicit BfsAlgorithm(IsWalkableFn is_walkable);

    void setConnectivity(Connectivity c) { connectivity_ = c; }
    void setGoalTolerance(double t) { goal_tolerance_ = t; }
    void setMaxIterations(int m) { max_iterations_ = m; }
    void setMaxPlanningTime(double t) { max_planning_time_ = t; }

    std::optional<std::vector<Vec3>> findPath(const Vec3& start, const Vec3& goal);
    std::optional<std::vector<Vec3>> findPathToAny(const Vec3& start, IsGoalFn is_goal);

private:
    std::vector<Vec3> getNeighbors(const Vec3& pos) const;
    static double cellDistance(const Vec3& a, const Vec3& b);
    std::vector<Vec3> reconstructPath(const Vec3& goal) const;

    IsWalkableFn is_walkable_;

    Connectivity connectivity_ = Connectivity::N8;
    double goal_tolerance_ = 0.25;
    int max_iterations_ = 1000000;
    double max_planning_time_ = 5.0;

    std::queue<Vec3> frontier_;
    std::unordered_set<Vec3, Vec3Hash> visited_;
    std::unordered_map<Vec3, Vec3, Vec3Hash> parent_;
};

} // namespace bfs
