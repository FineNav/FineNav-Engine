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
// Internal cell type is always Vec3.
//   • 2-D search: z = 0 for all cells, neighbours expand in xy only.
//   • 3-D search: full 6/18/26 neighbourhood.
//
// Map queries are injected as std::function at construction — the
// algorithm knows nothing about maps, world coordinates, or any
// IMapView interface.  The caller (AstarPathSearch) builds these
// lambdas from the user's IMapView and its own resolution/origin.
//
// All search parameters are set via setters before findPath().
// =============================================================

class AStarAlgorithm {
public:

    enum class Dimension { D2, D3 };

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

    void setDimension(Dimension d)    { dimension_        = d; }
    void setHeuristic(Heuristic h)    { heuristic_        = h; }
    void setDiagonalWeight(double w)  { diagonal_weight_  = w; }
    void setAllowDiagonal(bool a)     { allow_diagonal_   = a; }
    void setGoalTolerance(double t)   { goal_tolerance_   = t; }
    void setMaxIterations(int m)      { max_iterations_   = m; }
    void setMaxPlanningTime(double t) { max_planning_time_= t; }

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
    // D2: 4 ortho (xy, z=0 fixed)  + 4 diagonal if allowed
    // D3: 6 face  + 12 edge-diag   + 8 corner-diag if allowed

    std::vector<Vec3> getNeighbors(const Vec3& pos) const
    {
        std::vector<Vec3> neighbors;

        if (dimension_ == Dimension::D2)
        {
            static const std::array<Vec3,4> ortho = {{
                {0,1,0},{1,0,0},{0,-1,0},{-1,0,0}
            }};
            for (const auto& off : ortho)
                neighbors.push_back(pos + off);

            if (allow_diagonal_)
            {
                static const std::array<Vec3,4> diag = {{
                    {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}
                }};
                for (const auto& off : diag)
                    neighbors.push_back(pos + off);
            }
        }
        else
        {
            static const std::array<Vec3,6> face = {{
                {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
            }};
            for (const auto& off : face)
                neighbors.push_back(pos + off);

            if (allow_diagonal_)
            {
                static const std::array<Vec3,12> edge = {{
                    {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
                    {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
                    {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1}
                }};
                for (const auto& off : edge)
                    neighbors.push_back(pos + off);

                static const std::array<Vec3,8> corner = {{
                    {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
                    {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
                }};
                for (const auto& off : corner)
                    neighbors.push_back(pos + off);
            }
        }

        return neighbors;
    }

    // ── step cost ─────────────────────────────────────────────
    // Straight(1 axis)=1.0  Edge-diag(2)=√2  Corner-diag(3)=√3

    double calculateStepCost(const Vec3& from, const Vec3& to) const
    {
        int nd = (from.x != to.x) + (from.y != to.y) + (from.z != to.z);
        if (nd == 1) return 1.0;
        if (nd == 2) return diagonal_weight_;
        return diagonal_weight_ * std::sqrt(1.5);   // √3 = √2 * √1.5
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

    Dimension dimension_        = Dimension::D2;
    Heuristic heuristic_        = Heuristic::Euclidean;
    bool      allow_diagonal_   = true;
    double    diagonal_weight_  = std::sqrt(2.0);
    double    goal_tolerance_   = 0.25;
    int       max_iterations_   = 1000000;
    double    max_planning_time_= 5.0;

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

