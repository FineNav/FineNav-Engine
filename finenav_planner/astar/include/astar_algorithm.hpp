// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <vector>
#include <algorithm>
#include <array>
#include <type_traits>
#include <functional>
#include <limits>
#include <cmath>
#include <chrono>

#include "vec2.hpp"
#include "node_2d.hpp"
#include "heuristic.hpp"
#include "grid_map.hpp"

namespace astar {

template<typename Map>
class AStarAlgorithm {
public:

    AStarAlgorithm(
        const Map& map,
        Heuristic heuristic = Heuristic::Euclidean)
        : map_(map),
          heuristic_(heuristic)
    {}

    // =========================================================
    // setters
    // =========================================================

    void setDiagonalWeight(double weight)
    {
        diagonal_weight_ = weight;
    }

    void setAllowDiagonal(bool allow)
    {
        allow_diagonal_ = allow;
    }

    void setGoalTolerance(double tol)
    {
        goal_tolerance_ = tol;
    }

    void setAllowUnknown(bool allow)
    {
        allow_unknown_ = allow;
    }

    void setMaxIterations(int max_iter)
    {
        max_iterations_ = max_iter;
    }

    void setMaxPlanningTime(double max_time)
    {
        max_planning_time_ = max_time;
    }

    // =========================================================
    // main search
    // =========================================================

    std::optional<std::vector<Vec2>> findPath(
        const Vec2& start,
        const Vec2& goal)
    {
        // =====================================================
        // start / goal validity
        // =====================================================

        if (!invoke_isWalkable(map_, start) ||
            !invoke_isWalkable(map_, goal))
        {
            return std::nullopt;
        }

        // =====================================================
        // reset containers
        // =====================================================

        open_set_ = {};

        visited_.clear();

        came_from_.clear();

        int iterations = 0;

        auto start_time =
            std::chrono::steady_clock::now();

        // =====================================================
        // best near-goal candidate
        // =====================================================

        std::optional<Node2D<double>>
            best_goal_node;

        double best_goal_dist =
            std::numeric_limits<double>::infinity();

        // =====================================================
        // init start node
        // =====================================================

        Node2D<double> start_node(start);

        start_node.g = 0.0;

        start_node.h =
            calculateHeuristic(
                heuristic_,
                start,
                goal);

        start_node.updateF();

        // important:
        // let start parent point to itself
        start_node.parent = start;

        open_set_.push(start_node);

        came_from_[start] = start_node;

        // =====================================================
        // main loop
        // =====================================================

        while (!open_set_.empty())
        {
            // =================================================
            // max iterations limit
            // =================================================

            if (max_iterations_ >= 0 &&
                iterations++ > max_iterations_)
            {
                break;
            }

            // =================================================
            // planning timeout
            // =================================================

            auto now =
                std::chrono::steady_clock::now();

            double elapsed =
                std::chrono::duration<double>(
                    now - start_time).count();

            if (elapsed > max_planning_time_)
            {
                break;
            }

            // =================================================
            // get current node
            // =================================================

            Node2D<double> current =
                open_set_.top();

            open_set_.pop();

            // =================================================
            // skip closed nodes
            // =================================================

            if (visited_.contains(current.pos))
            {
                continue;
            }

            visited_.insert(current.pos);

            // =================================================
            // exact goal reached
            // =================================================

            if (current.pos == goal)
            {
                return reconstructPath(goal);
            }

            // =================================================
            // update best near-goal node
            // =================================================

            double goal_dist =
                distanceToGoal(
                    current.pos,
                    goal);

            if (goal_dist < best_goal_dist)
            {
                best_goal_dist =
                    goal_dist;

                best_goal_node =
                    current;
            }

            // =================================================
            // expand neighbors
            // =================================================

            for (const auto& neighbor_pos :
                 getNeighbors(current.pos))
            {
                // ---------------------------------------------
                // unknown handling
                // ---------------------------------------------

                if (!allow_unknown_ &&
                    invoke_isUnknown(
                        map_,
                        neighbor_pos))
                {
                    continue;
                }

                // ---------------------------------------------
                // obstacle handling
                // ---------------------------------------------

                if (!invoke_isWalkable(
                        map_,
                        neighbor_pos))
                {
                    continue;
                }

                // ---------------------------------------------
                // closed set skip
                // ---------------------------------------------

                if (visited_.contains(
                        neighbor_pos))
                {
                    continue;
                }

                // ---------------------------------------------
                // movement cost
                // ---------------------------------------------

                double step_cost =
                    calculateStepCost(
                        current.pos,
                        neighbor_pos);

                // ---------------------------------------------
                // map traversal cost
                // IMPORTANT:
                // avoid multiplying by zero
                // ---------------------------------------------

                double map_cost =
                    invoke_getCost(
                        map_,
                        neighbor_pos);

                double traversal_cost =
                    1.0 + map_cost;

                double tentative_g =
                    current.g +
                    step_cost *
                    traversal_cost;

                // ---------------------------------------------
                // relaxation
                // ---------------------------------------------

                if (!came_from_.contains(
                        neighbor_pos)
                    ||
                    tentative_g <
                    came_from_[neighbor_pos].g)
                {
                    Node2D<double> neighbor(
                        neighbor_pos);

                    neighbor.g =
                        tentative_g;

                    neighbor.h =
                        calculateHeuristic(
                            heuristic_,
                            neighbor_pos,
                            goal);

                    // -----------------------------------------
                    // tie breaker
                    // encourage straighter path
                    // -----------------------------------------

                    neighbor.h *= 1.001;

                    neighbor.updateF();

                    neighbor.parent =
                        current.pos;

                    came_from_[neighbor_pos] =
                        neighbor;

                    open_set_.push(neighbor);
                }
            }
        }

        // =====================================================
        // fallback:
        // return best near-goal path
        // =====================================================

        if (best_goal_node.has_value() &&
            best_goal_dist <= goal_tolerance_)
        {
            return reconstructPath(
                best_goal_node->pos);
        }

        return std::nullopt;
    }

private:

    // =========================================================
    // helper:
    // walkability
    // =========================================================

    static bool invoke_isWalkable(
        const Map& m,
        const Vec2& pos)
    {
        if constexpr (
            requires(const Map& mm,
                     const Vec2& p)
            {
                mm.isWalkable(p);
            })
        {
            return m.isWalkable(pos);
        }
        else if constexpr (
            std::is_invocable_v<
                Map,
                const Vec2&>)
        {
            double c =
                std::invoke(m, pos);

            return std::isfinite(c) &&
                   c <
                   std::numeric_limits<double>::infinity();
        }
        else
        {
            return false;
        }
    }

    // =========================================================
    // helper:
    // unknown
    // =========================================================

    static bool invoke_isUnknown(
        const Map& m,
        const Vec2& pos)
    {
        if constexpr (
            requires(const Map& mm,
                     const Vec2& p)
            {
                mm.isUnknown(p);
            })
        {
            return m.isUnknown(pos);
        }
        else
        {
            return false;
        }
    }

    // =========================================================
    // helper:
    // traversal cost
    // =========================================================

    static double invoke_getCost(
        const Map& m,
        const Vec2& pos)
    {
        if constexpr (
            requires(const Map& mm,
                     const Vec2& p)
            {
                mm.getCost(p);
            })
        {
            return m.getCost(pos);
        }
        else if constexpr (
            std::is_invocable_v<
                Map,
                const Vec2&>)
        {
            return std::invoke(m, pos);
        }
        else
        {
            return std::numeric_limits<double>::infinity();
        }
    }

    // =========================================================
    // helper:
    // goal distance
    // =========================================================

    double distanceToGoal(
        const Vec2& current,
        const Vec2& goal) const
    {
        double dx =
            static_cast<double>(
                current.x - goal.x);

        double dy =
            static_cast<double>(
                current.y - goal.y);

        return std::sqrt(dx * dx + dy * dy);
    }

    // =========================================================
    // movement cost
    // =========================================================

    double calculateStepCost(
        const Vec2& from,
        const Vec2& to) const
    {
        int dx =
            std::abs(from.x - to.x);

        int dy =
            std::abs(from.y - to.y);

        return (dx == 1 && dy == 1)
               ? diagonal_weight_
               : 1.0;
    }

    // =========================================================
    // neighbors
    // =========================================================

    std::vector<Vec2> getNeighbors(
        const Vec2& pos) const
    {
        std::vector<Vec2> neighbors;

        const std::array<Vec2, 4> ortho =
        {
            Vec2{0,1},
            Vec2{1,0},
            Vec2{0,-1},
            Vec2{-1,0}
        };

        for (const auto& off : ortho)
        {
            neighbors.push_back(
                pos + off);
        }

        if (allow_diagonal_)
        {
            const std::array<Vec2, 4> diag =
            {
                Vec2{1,1},
                Vec2{1,-1},
                Vec2{-1,1},
                Vec2{-1,-1}
            };

            for (const auto& off : diag)
            {
                neighbors.push_back(
                    pos + off);
            }
        }

        return neighbors;
    }

    // =========================================================
    // reconstruct path
    // =========================================================

    std::vector<Vec2> reconstructPath(
        const Vec2& goal) const
    {
        std::vector<Vec2> path;

        Vec2 cur = goal;

        while (true)
        {
            path.push_back(cur);

            const auto& node =
                came_from_.at(cur);

            if (cur == node.parent)
            {
                break;
            }

            cur = node.parent;
        }

        std::reverse(
            path.begin(),
            path.end());

        return path;
    }

private:

    // =========================================================
    // map
    // =========================================================

    const Map& map_;

    // =========================================================
    // planner config
    // =========================================================

    Heuristic heuristic_ =
        Heuristic::Euclidean;

    bool allow_diagonal_ = true;

    double diagonal_weight_ =
        std::sqrt(2.0);

    bool allow_unknown_ = true;

    double goal_tolerance_ = 0.25;

    int max_iterations_ = 1000000;

    double max_planning_time_ = 5.0;

    // =========================================================
    // search containers
    // =========================================================

    std::priority_queue<
        Node2D<double>,
        std::vector<Node2D<double>>,
        Node2DCompare<double>
    > open_set_;

    std::unordered_set<
        Vec2,
        Vec2Hash
    > visited_;

    std::unordered_map<
        Vec2,
        Node2D<double>,
        Vec2Hash
    > came_from_;
};

} // namespace astar

// =============================================================
// backwards compatibility
// =============================================================

namespace astar {

template<typename Map>
using AStar = AStarAlgorithm<Map>;

template<typename T = double>
using Node = Node2D<T>;

template<typename T>
using NodeCompare = Node2DCompare<T>;

}