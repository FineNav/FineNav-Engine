// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <vector>
#include <algorithm>
#include <array>
#include <limits>
#include <cmath>
#include <chrono>

#include "vec3.hpp"
#include "heuristic.hpp"

namespace astar {

// =============================================================
// AstarNode  —  internal grid search node, always Vec3-based
// =============================================================

struct AstarNode {
    Vec3   pos;
    double g = std::numeric_limits<double>::max();
    double h = 0.0;
    double f = std::numeric_limits<double>::max();
    Vec3   parent{};   // sentinel: parent == pos means this is the start

    AstarNode() = default;
    explicit AstarNode(const Vec3& p) : pos(p) {}
    void updateF() { f = g + h; }
};

struct AstarNodeCompare {
    bool operator()(const AstarNode& a, const AstarNode& b) const
    { return a.f > b.f; }
};

// =============================================================
// AStarAlgorithm
//
// A plain, non-template uniform-grid A* engine.
//
// Neighbourhood is controlled by Connectivity:
//   N4  — 2-D,  4 orthogonal neighbours
//   N8  — 2-D,  8 neighbours  (4 ortho + 4 diagonal)
//   N6  — 3-D,  6 face neighbours
//   N18 — 3-D, 18 neighbours  (6 face + 12 edge-diagonal)
//   N26 — 3-D, 26 neighbours  (18 + 8 corner-diagonal)
//
// Step costs are pure geometry — not user-configurable:
//   1-axis step  →  1.0
//   2-axis step  →  √2  ≈ 1.4142
//   3-axis step  →  √3  ≈ 1.7321
//
// Map queries are injected as std::function at construction — the
// algorithm knows nothing about maps, world coordinates, or any
// IMapView interface.
//
// All search parameters are set via setters before findPath().
// =============================================================

class AStarAlgorithm {
public:

    // ── Connectivity: encodes both dimension and neighbourhood size ──────
    //
    //   N4  — 2-D, 4-connected  (orthogonal only)
    //   N8  — 2-D, 8-connected  (orthogonal + diagonal)
    //   N6  — 3-D, 6-connected  (face neighbours only)
    //   N18 — 3-D, 18-connected (face + edge-diagonal)
    //   N26 — 3-D, 26-connected (face + edge-diagonal + corner-diagonal)
    enum class Connectivity { N4, N8, N6, N18, N26 };

    using IsWalkableFn = std::function<bool(const Vec3&)>;
    using GetCostFn    = std::function<double(const Vec3&)>;

    // Supply the two grid-query lambdas.
    AStarAlgorithm(
        IsWalkableFn is_walkable,
        GetCostFn    get_cost)
        : is_walkable_(std::move(is_walkable)),
          get_cost_(std::move(get_cost))
    {}

    // ── setters ───────────────────────────────────────────────

    void setConnectivity(Connectivity c)  { connectivity_     = c; }
    void setHeuristic(Heuristic h)        { heuristic_        = h; }
    void setGoalTolerance(double t)       { goal_tolerance_   = t; }
    void setMaxIterations(int m)          { max_iterations_   = m; }
    void setMaxPlanningTime(double t)     { max_planning_time_= t; }

    // ── main search ───────────────────────────────────────────

    std::optional<std::vector<Vec3>> findPath(
        const Vec3& start,
        const Vec3& goal)
    {
        if (!is_walkable_(start) || !is_walkable_(goal))
            return std::nullopt;

        open_set_ = {};
        visited_.clear();
        came_from_.clear();

        int  iterations = 0;
        auto start_time = std::chrono::steady_clock::now();

        std::optional<AstarNode> best_goal_node;
        double best_goal_dist = std::numeric_limits<double>::infinity();

        AstarNode start_node(start);
        start_node.g      = 0.0;
        start_node.h      = calculateHeuristic(heuristic_, start, goal);
        start_node.updateF();
        start_node.parent = start;   // self-sentinel

        open_set_.push(start_node);
        came_from_[start] = start_node;

        while (!open_set_.empty())
        {
            if (max_iterations_ >= 0 &&
                iterations++ > max_iterations_)
            { break; }

            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed > max_planning_time_) { break; }

            AstarNode current = open_set_.top();
            open_set_.pop();

            if (visited_.contains(current.pos)) continue;
            visited_.insert(current.pos);

            if (current.pos == goal)
                return reconstructPath(goal);

            double goal_dist = cellDistance(current.pos, goal);
            if (goal_dist < best_goal_dist)
            {
                best_goal_dist = goal_dist;
                best_goal_node = current;
            }

            for (const Vec3& nb_pos : getNeighbors(current.pos))
            {
                if (!is_walkable_(nb_pos))                   continue;
                if (visited_.contains(nb_pos))               continue;

                double step_cost   = calculateStepCost(current.pos, nb_pos);
                double map_cost    = get_cost_(nb_pos);
                double tentative_g = current.g + step_cost * (1.0 + map_cost);

                if (!came_from_.contains(nb_pos) ||
                    tentative_g < came_from_[nb_pos].g)
                {
                    AstarNode nb(nb_pos);
                    nb.g      = tentative_g;
                    nb.h      = calculateHeuristic(heuristic_, nb_pos, goal) * 1.001;
                    nb.updateF();
                    nb.parent = current.pos;

                    came_from_[nb_pos] = nb;
                    open_set_.push(nb);
                }
            }
        }

        if (best_goal_node.has_value() &&
            best_goal_dist <= goal_tolerance_)
        {
            return reconstructPath(best_goal_node->pos);
        }

        return std::nullopt;
    }

private:

    // ── neighbour generation ──────────────────────────────────
    //
    // Offsets are compile-time constants; switch dispatches on connectivity_.

    std::vector<Vec3> getNeighbors(const Vec3& pos) const
    {
        static const std::array<Vec3, 4> ortho2d = {{
            {1,0,0},{-1,0,0},{0,1,0},{0,-1,0}
        }};
        static const std::array<Vec3, 4> diag2d = {{
            {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}
        }};
        static const std::array<Vec3, 6> face3d = {{
            {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
        }};
        static const std::array<Vec3, 12> edge3d = {{
            {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
            {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
            {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1}
        }};
        static const std::array<Vec3, 8> corner3d = {{
            {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
            {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
        }};

        std::vector<Vec3> neighbors;

        switch (connectivity_)
        {
            case Connectivity::N4:
                for (const auto& o : ortho2d)  neighbors.push_back(pos + o);
                break;

            case Connectivity::N8:
                for (const auto& o : ortho2d)  neighbors.push_back(pos + o);
                for (const auto& o : diag2d)   neighbors.push_back(pos + o);
                break;

            case Connectivity::N6:
                for (const auto& o : face3d)   neighbors.push_back(pos + o);
                break;

            case Connectivity::N18:
                for (const auto& o : face3d)   neighbors.push_back(pos + o);
                for (const auto& o : edge3d)   neighbors.push_back(pos + o);
                break;

            case Connectivity::N26:
                for (const auto& o : face3d)   neighbors.push_back(pos + o);
                for (const auto& o : edge3d)   neighbors.push_back(pos + o);
                for (const auto& o : corner3d) neighbors.push_back(pos + o);
                break;
        }

        return neighbors;
    }

    // ── step cost — pure geometry, never user-configurable ────
    //   1 axis changed  →  1.0
    //   2 axes changed  →  √2
    //   3 axes changed  →  √3

    static double calculateStepCost(const Vec3& from, const Vec3& to)
    {
        int nd = (from.x != to.x) + (from.y != to.y) + (from.z != to.z);
        if (nd <= 1) return 1.0;
        if (nd == 2) return std::sqrt(2.0);
        return std::sqrt(3.0);
    }

    static double cellDistance(const Vec3& a, const Vec3& b)
    {
        double dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    std::vector<Vec3> reconstructPath(const Vec3& goal) const
    {
        std::vector<Vec3> path;
        Vec3 cur = goal;
        while (true)
        {
            path.push_back(cur);
            const auto& node = came_from_.at(cur);
            if (cur == node.parent) break;
            cur = node.parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // ── injected queries ──────────────────────────────────────

    IsWalkableFn is_walkable_;
    GetCostFn    get_cost_;

    // ── parameters ────────────────────────────────────────────

    Connectivity connectivity_   = Connectivity::N8;
    Heuristic    heuristic_      = Heuristic::Euclidean;
    double       goal_tolerance_ = 0.25;
    int          max_iterations_ = 1000000;
    double       max_planning_time_ = 5.0;

    // ── search state (reset each call) ────────────────────────

    std::priority_queue<
        AstarNode,
        std::vector<AstarNode>,
        AstarNodeCompare
    > open_set_;

    std::unordered_set<Vec3, Vec3Hash>             visited_;
    std::unordered_map<Vec3, AstarNode, Vec3Hash>  came_from_;
};

} // namespace astar

