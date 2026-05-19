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

#include "vec2.hpp"
#include "node_2d.hpp"
#include "heuristic.hpp"
#include "grid_map.hpp"   // optional default map implementation

namespace astar {

// A flexible A* implementation that accepts either:
//  - a map-like object with `isWalkable(const Vec2&)` and `getCost(const Vec2&)`, OR
//  - a callable Map: double(const Vec2&) returning cost (use infinity for non-walkable)
template<typename Map>
class AStarAlgorithm {
public:
    AStarAlgorithm(const Map& map, Heuristic heuristic = Heuristic::Euclidean)
        : map_(map), heuristic_(heuristic) {}

    void setDiagonalWeight(double weight) { diagonal_weight_ = weight; }
    void setAllowDiagonal(bool allow) { allow_diagonal_ = allow; }

    std::optional<std::vector<Vec2>> findPath(const Vec2& start, const Vec2& goal) {
        if (!invoke_isWalkable(map_, start) || !invoke_isWalkable(map_, goal))
            return std::nullopt;

        open_set_ = {};
        visited_.clear();
        came_from_.clear();

        Node2D<double> start_node(start);
        start_node.g = 0;
        start_node.h = calculateHeuristic(heuristic_, start, goal);
        start_node.updateF();

        open_set_.push(start_node);
        came_from_[start] = start_node;

        while (!open_set_.empty()) {
            Node2D<double> current = open_set_.top();
            open_set_.pop();

            if (current.pos == goal)
                return reconstructPath(goal);

            if (visited_.contains(current.pos))
                continue;
            visited_.insert(current.pos);

            for (const auto& neighbor_pos : getNeighbors(current.pos)) {
                if (!invoke_isWalkable(map_, neighbor_pos))
                    continue;

                double step_cost = calculateStepCost(current.pos, neighbor_pos);
                step_cost *= invoke_getCost(map_, neighbor_pos);
                double tentative_g = current.g + step_cost;

                if (!came_from_.contains(neighbor_pos) || tentative_g < came_from_[neighbor_pos].g) {
                    Node2D<double> neighbor(neighbor_pos);
                    neighbor.g = tentative_g;
                    neighbor.h = calculateHeuristic(heuristic_, neighbor_pos, goal);
                    neighbor.updateF();
                    neighbor.parent = current.pos;

                    came_from_[neighbor_pos] = neighbor;
                    open_set_.push(neighbor);
                }
            }
        }

        return std::nullopt;
    }

private:
    // Helper to check walkability for either map-like objects or callables
    static bool invoke_isWalkable(const Map& m, const Vec2& pos) {
        if constexpr (requires(const Map& mm, const Vec2& p) { mm.isWalkable(p); }) {
            return m.isWalkable(pos);
        } else if constexpr (std::is_invocable_v<Map, const Vec2&>) {
            double c = std::invoke(m, pos);
            return std::isfinite(c) && c < std::numeric_limits<double>::infinity();
        } else {
            return false;
        }
    }

    // Helper to get cost for either map-like objects or callables
    static double invoke_getCost(const Map& m, const Vec2& pos) {
        if constexpr (requires(const Map& mm, const Vec2& p) { mm.getCost(p); }) {
            return m.getCost(pos);
        } else if constexpr (std::is_invocable_v<Map, const Vec2&>) {
            return std::invoke(m, pos);
        } else {
            return std::numeric_limits<double>::infinity();
        }
    }

    double calculateStepCost(const Vec2& from, const Vec2& to) const {
        int dx = std::abs(from.x - to.x);
        int dy = std::abs(from.y - to.y);
        return (dx == 1 && dy == 1) ? diagonal_weight_ : 1.0;
    }

    std::vector<Vec2> getNeighbors(const Vec2& pos) const {
        std::vector<Vec2> neighbors;
        const std::array<Vec2, 4> ortho = {Vec2{0,1}, Vec2{1,0}, Vec2{0,-1}, Vec2{-1,0}};
        for (const auto& off : ortho) {
            Vec2 np = pos + off;
            neighbors.push_back(np);
        }

        if (allow_diagonal_) {
            const std::array<Vec2,4> diag = {Vec2{1,1}, Vec2{1,-1}, Vec2{-1,1}, Vec2{-1,-1}};
            for (const auto& off : diag) {
                Vec2 np = pos + off;
                neighbors.push_back(np);
            }
        }
        return neighbors;
    }

    std::vector<Vec2> reconstructPath(const Vec2& goal) const {
        std::vector<Vec2> path;
        Vec2 cur = goal;
        // protect against pathological loops by bounding to known visited/came_from size
        size_t guard = came_from_.size() ? came_from_.size() : static_cast<size_t>(100000);
        while (came_from_.contains(cur) && path.size() <= guard) {
            path.push_back(cur);
            cur = came_from_.at(cur).parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    const Map& map_;
    Heuristic heuristic_;
    bool allow_diagonal_ = true;
    double diagonal_weight_ = std::sqrt(2);

    std::priority_queue<Node2D<double>, std::vector<Node2D<double>>, Node2DCompare<double>> open_set_;
    std::unordered_set<Vec2, Vec2Hash> visited_;
    std::unordered_map<Vec2, Node2D<double>, Vec2Hash> came_from_;
};

} // namespace astar

// Backwards-compatibility type aliases
namespace astar {
    template<typename Map>
    using AStar = AStarAlgorithm<Map>;

    template<typename T = double>
    using Node = Node2D<T>;

    template<typename T>
    using NodeCompare = Node2DCompare<T>;
}

