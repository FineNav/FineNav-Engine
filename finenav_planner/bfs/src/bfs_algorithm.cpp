// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "bfs_algorithm.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

namespace bfs {

BfsAlgorithm::BfsAlgorithm(IsWalkableFn is_walkable)
    : is_walkable_(std::move(is_walkable))
{}

std::optional<std::vector<Vec3>> BfsAlgorithm::findPath(
    const Vec3& start,
    const Vec3& goal)
{
    if (!is_walkable_(start) || !is_walkable_(goal)) {
        return std::nullopt;
    }

    frontier_ = {};
    visited_.clear();
    parent_.clear();

    int iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    Vec3 best_goal = start;
    double best_goal_dist = std::numeric_limits<double>::infinity();

    frontier_.push(start);
    visited_.insert(start);
    parent_[start] = start;

    while (!frontier_.empty())
    {
        if (max_iterations_ >= 0 && iterations++ >= max_iterations_) {
            break;
        }

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_) {
            break;
        }

        const Vec3 current = frontier_.front();
        frontier_.pop();

        if (current == goal) {
            return reconstructPath(goal);
        }

        const double goal_dist = cellDistance(current, goal);
        if (goal_dist < best_goal_dist) {
            best_goal_dist = goal_dist;
            best_goal = current;
        }

        for (const Vec3& nb_pos : getNeighbors(current))
        {
            if (!is_walkable_(nb_pos)) {
                continue;
            }
            if (visited_.contains(nb_pos)) {
                continue;
            }

            visited_.insert(nb_pos);
            parent_[nb_pos] = current;
            frontier_.push(nb_pos);
        }
    }

    if (best_goal_dist <= goal_tolerance_) {
        return reconstructPath(best_goal);
    }

    return std::nullopt;
}

std::optional<std::vector<Vec3>> BfsAlgorithm::findPathToAny(
    const Vec3& start,
    IsGoalFn is_goal)
{
    if (!is_walkable_(start)) {
        return std::nullopt;
    }

    frontier_ = {};
    visited_.clear();
    parent_.clear();

    int iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    frontier_.push(start);
    visited_.insert(start);
    parent_[start] = start;

    while (!frontier_.empty())
    {
        if (max_iterations_ >= 0 && iterations++ >= max_iterations_) {
            break;
        }

        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_) {
            break;
        }

        const Vec3 current = frontier_.front();
        frontier_.pop();

        if (is_goal(current)) {
            return reconstructPath(current);
        }

        for (const Vec3& nb_pos : getNeighbors(current))
        {
            if (!is_walkable_(nb_pos)) {
                continue;
            }
            if (visited_.contains(nb_pos)) {
                continue;
            }

            visited_.insert(nb_pos);
            parent_[nb_pos] = current;
            frontier_.push(nb_pos);
        }
    }

    return std::nullopt;
}

std::vector<Vec3> BfsAlgorithm::getNeighbors(const Vec3& pos) const
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
            for (const auto& o : ortho2d) neighbors.push_back(pos + o);
            break;

        case Connectivity::N8:
            for (const auto& o : ortho2d) neighbors.push_back(pos + o);
            for (const auto& o : diag2d) neighbors.push_back(pos + o);
            break;

        case Connectivity::N6:
            for (const auto& o : face3d) neighbors.push_back(pos + o);
            break;

        case Connectivity::N18:
            for (const auto& o : face3d) neighbors.push_back(pos + o);
            for (const auto& o : edge3d) neighbors.push_back(pos + o);
            break;

        case Connectivity::N26:
            for (const auto& o : face3d) neighbors.push_back(pos + o);
            for (const auto& o : edge3d) neighbors.push_back(pos + o);
            for (const auto& o : corner3d) neighbors.push_back(pos + o);
            break;
    }

    return neighbors;
}

double BfsAlgorithm::cellDistance(const Vec3& a, const Vec3& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<Vec3> BfsAlgorithm::reconstructPath(const Vec3& goal) const
{
    std::vector<Vec3> path;
    Vec3 cur = goal;
    while (true)
    {
        path.push_back(cur);
        const Vec3 p = parent_.at(cur);
        if (cur == p) {
            break;
        }
        cur = p;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace bfs
