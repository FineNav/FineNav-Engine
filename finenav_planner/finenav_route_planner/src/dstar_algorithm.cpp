// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/dstar_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace astar {

// ===================================================================
// Construction
// ===================================================================

DStarAlgorithm::DStarAlgorithm(int rows, int cols,
                               IsWalkableFn is_walkable,
                               GetCostFn    get_cost,
                               Connectivity connectivity,
                               Heuristic    heuristic,
                               int          layers)
    : is_walkable_(std::move(is_walkable))
    , get_cost_(std::move(get_cost))
    , rows_(rows)
    , cols_(cols)
    , layers_(layers)
    , connectivity_(connectivity)
    , heuristic_type_(heuristic)
{
    if (rows <= 0 || cols <= 0 || layers <= 0) {
        throw std::invalid_argument(
            "Grid dimensions must be positive (got " +
            std::to_string(rows) + "x" + std::to_string(cols) +
            "x" + std::to_string(layers) + ")");
    }

    const auto size = static_cast<size_t>(rows_ * cols_ * layers_);
    g_.assign(size, kInf);
    rhs_.assign(size, kInf);
    obstacles_.assign(size, false);
}

// ===================================================================
// Connectivity / heuristic setters
// ===================================================================

void DStarAlgorithm::setConnectivity(Connectivity c) {
    connectivity_ = c;
}

void DStarAlgorithm::setHeuristic(Heuristic h) {
    heuristic_type_ = h;
}

// ===================================================================
// Grid queries
// ===================================================================

bool DStarAlgorithm::isWalkable(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return is_walkable_(p);
}

double DStarAlgorithm::getCost(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return get_cost_(p);
}

// ===================================================================
// Obstacle modification
// ===================================================================

void DStarAlgorithm::setObstacle(const Vec3& p) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    if (obstacles_[idx(p)]) return;
    obstacles_[idx(p)] = true;

    if (!valid_) return;

    g_[idx(p)]   = kInf;
    rhs_[idx(p)] = kInf;
    key_cache_.erase(p);

    for (const auto& nb : getAllNeighbors(p)) {
        if (!obstacles_[idx(nb)]) {
            updateVertex(nb);
        }
    }
}

void DStarAlgorithm::removeObstacle(const Vec3& p) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    if (!obstacles_[idx(p)]) return;
    obstacles_[idx(p)] = false;

    if (!valid_) return;

    updateVertex(p);

    for (const auto& nb : getAllNeighbors(p)) {
        if (!obstacles_[idx(nb)]) {
            updateVertex(nb);
        }
    }
}

// ===================================================================
// Map synchronisation
// ===================================================================

void DStarAlgorithm::syncMap() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (int z = 0; z < layers_; ++z) {
     for (int y = 0; y < rows_; ++y) {
      for (int x = 0; x < cols_; ++x) {
            const Vec3 p{x, y, z};
            bool real_obstacle = !is_walkable_(p);
            if (real_obstacle != obstacles_[idx(p)]) {
                if (real_obstacle) {
                    obstacles_[idx(p)] = true;
                    if (!valid_) continue;
                    g_[idx(p)]   = kInf;
                    rhs_[idx(p)] = kInf;
                    key_cache_.erase(p);
                    for (const auto& nb : getAllNeighbors(p)) {
                        if (!obstacles_[idx(nb)]) updateVertex(nb);
                    }
                } else {
                    obstacles_[idx(p)] = false;
                    if (!valid_) continue;
                    updateVertex(p);
                    for (const auto& nb : getAllNeighbors(p)) {
                        if (!obstacles_[idx(nb)]) updateVertex(nb);
                    }
                    }
                }
            }
        }
    }
}

// ===================================================================
// Algorithm lifecycle
// ===================================================================

void DStarAlgorithm::initialize(const Vec3& start, const Vec3& goal) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(start);
    checkBounds(goal);

    // Sync obstacles from the map callback (3-D).
    for (int z = 0; z < layers_; ++z) {
     for (int y = 0; y < rows_; ++y) {
      for (int x = 0; x < cols_; ++x) {
            const Vec3 p{x, y, z};
            obstacles_[idx(p)] = !is_walkable_(p);
      }
     }
    }

    std::fill(g_.begin(), g_.end(), kInf);
    std::fill(rhs_.begin(), rhs_.end(), kInf);

    U_ = decltype(U_){};
    key_cache_.clear();

    start_ = start;

    // ── goal tolerance: find nearest walkable cell if goal is blocked ─
    Vec3 effective_goal = goal;
    if (!is_walkable_(goal) && goal_tolerance_ > 0) {
        int best_dist = goal_tolerance_ + 1;
        for (int dz = -goal_tolerance_; dz <= goal_tolerance_; ++dz) {
         for (int dy = -goal_tolerance_; dy <= goal_tolerance_; ++dy) {
          for (int dx = -goal_tolerance_; dx <= goal_tolerance_; ++dx) {
                int nz = goal.z + dz;
                int ny = goal.y + dy;
                int nx = goal.x + dx;
                if (nz < 0 || nz >= layers_ ||
                    ny < 0 || ny >= rows_ ||
                    nx < 0 || nx >= cols_) continue;
                int dist = std::abs(dz) + std::abs(dy) + std::abs(dx);
                const Vec3 candidate{nx, ny, nz};
                if (dist <= goal_tolerance_ && dist < best_dist &&
                    is_walkable_(candidate)) {
                    best_dist = dist;
                    effective_goal = candidate;
                }
          }
         }
        }
        if (best_dist > goal_tolerance_) {
            std::cerr << "[DStarAlgorithm] WARN: goal (" << goal.x
                      << "," << goal.y << ") blocked and no walkable cell"
                      << " within tolerance " << goal_tolerance_ << ".\n";
        }
    }

    goal_ = effective_goal;
    km_   = 0.0;

    rhs_[idx(goal_)] = 0.0;
    insertOrUpdate(goal_);

    valid_ = true;
}

void DStarAlgorithm::computeShortestPath() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    int expansions = 0;

    const auto t_start = (max_planning_time_ > 0.0)
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};

    while (!U_.empty()) {
        // ── expansion limit ─────────────────────────────────
        if (max_expansions_ > 0 && ++expansions > max_expansions_) {
            std::cerr << "[DStarAlgorithm] ERROR: computeShortestPath "
                      << "exceeded " << max_expansions_
                      << " expansions — aborting.\n";
            break;
        }

        // ── time limit ──────────────────────────────────────
        if (max_planning_time_ > 0.0) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start);
            if (elapsed.count() > max_planning_time_) {
                std::cerr << "[DStarAlgorithm] WARN: computeShortestPath "
                          << "exceeded " << max_planning_time_
                          << " s time limit — aborting.\n";
                break;
            }
        }

        const auto [top_key, top_node] = U_.top();

        // Termination: start is consistent and has optimal key.
        const Key start_key = calculateKey(start_);
        if (!(top_key < start_key) &&
            !(rhs_[idx(start_)] > g_[idx(start_)])) {
            break;
        }

        U_.pop();

        // Lazy deletion: skip stale entries.
        auto it = key_cache_.find(top_node);
        if (it == key_cache_.end() || !(top_key == it->second)) {
            continue;
        }
        key_cache_.erase(top_node);

        const double g_u   = g_[idx(top_node)];
        const double rhs_u = rhs_[idx(top_node)];

        if (g_u > rhs_u) {
            // Over-consistent — cost improved.
            g_[idx(top_node)] = rhs_u;
            for (const auto& s : getPredecessors(top_node)) {
                updateVertex(s);
            }
        } else {
            // Under-consistent — cost worsened.
            g_[idx(top_node)] = kInf;
            updateVertex(top_node);
            for (const auto& s : getPredecessors(top_node)) {
                updateVertex(s);
            }
        }
    }
}

void DStarAlgorithm::replan(const Vec3& new_start) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(new_start);
    km_ += calculateHeuristic(heuristic_type_, start_, new_start);
    start_ = new_start;
    computeShortestPath();
}

bool DStarAlgorithm::step() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (start_ == goal_) return true;  // already there

    const auto neighbors = getSuccessors(start_);
    if (neighbors.empty()) return false;

    Vec3   best      = neighbors[0];
    double best_cost = kInf;

    for (const auto& nb : neighbors) {
        double cost_val = moveCost(start_, nb) + g_[idx(nb)];
        if (cost_val < best_cost) {
            best_cost = cost_val;
            best      = nb;
        }
    }

    if (best_cost >= kInf * 0.5) return false;

    start_ = best;   // NOTE: km is NOT updated here (paper spec).
    return true;
}

// ===================================================================
// Path extraction
// ===================================================================

std::vector<Vec3> DStarAlgorithm::extractPath() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<Vec3> path;
    Vec3 current = start_;
    path.push_back(current);

    int steps = 0;

    while (current != goal_ && (max_path_steps_ < 0 || steps < max_path_steps_)) {
        ++steps;

        const auto neighbors = getSuccessors(current);
        if (neighbors.empty()) break;

        Vec3   best     = neighbors[0];
        double best_val = kInf;

        for (const auto& nb : neighbors) {
            double val = moveCost(current, nb) + g_[idx(nb)];
            if (val < best_val) {
                best_val = val;
                best     = nb;
            }
        }

        if (best_val >= kInf * 0.5) break;

        current = best;
        path.push_back(current);
    }

    if (path.back() != goal_) return {};
    return path;
}

// ===================================================================
// Accessors
// ===================================================================

double DStarAlgorithm::g(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    return g_[idx(p)];
}

double DStarAlgorithm::rhs(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    return rhs_[idx(p)];
}

// ===================================================================
// Debug
// ===================================================================

void DStarAlgorithm::printGrid(const std::vector<Vec3>& path) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::unordered_map<Vec3, bool, Vec3Hash> on_path;
    for (const auto& p : path) on_path[p] = true;

    // Print layer-by-layer (for brevity, only z=0 is shown when layers_ > 1)
    int z_show = 0;
    if (layers_ > 1) {
        std::cout << "[DStarAlgorithm] " << layers_ << " layers; showing z=0.\n";
    }

    for (int y = 0; y < rows_; ++y) {
        for (int x = 0; x < cols_; ++x) {
            Vec3 p{x, y, z_show};
            if (p == start_)             std::cout << "S ";
            else if (p == goal_)         std::cout << "G ";
            else if (obstacles_[idx(p)]) std::cout << "# ";
            else if (on_path.contains(p)) std::cout << "* ";
            else                          std::cout << ". ";
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

// ===================================================================
// Core D* Lite functions
// ===================================================================

Key DStarAlgorithm::calculateKey(const Vec3& s) const {
    const double ming  = std::min(g_[idx(s)], rhs_[idx(s)]);
    const double h_val = calculateHeuristic(heuristic_type_, start_, s);
    return {ming + h_val + km_, ming};
}

void DStarAlgorithm::updateVertex(const Vec3& u) {
    // Blocked cells are unreachable.
    if (obstacles_[idx(u)]) {
        g_[idx(u)]   = kInf;
        rhs_[idx(u)] = kInf;
        key_cache_.erase(u);
        return;
    }

    if (u != goal_) {
        double best = kInf;
        for (const auto& sp : getSuccessors(u)) {
            double candidate = moveCost(u, sp) + g_[idx(sp)];
            if (candidate < best) best = candidate;
        }
        rhs_[idx(u)] = best;
    }

    auto it = key_cache_.find(u);
    if (it != key_cache_.end()) key_cache_.erase(it);

    if (g_[idx(u)] != rhs_[idx(u)]) {
        insertOrUpdate(u);
    }
}

void DStarAlgorithm::insertOrUpdate(const Vec3& s) {
    Key k = calculateKey(s);
    key_cache_[s] = k;
    U_.emplace(k, s);
}

// ===================================================================
// Neighbours & costs  (delegated to grid_search_utils.hpp)
// ===================================================================

std::vector<Vec3> DStarAlgorithm::getSuccessors(const Vec3& u) const {
    std::vector<Vec3> result;
    const auto neighbors = getNeighbors(u, connectivity_);
    result.reserve(neighbors.size());

    for (const auto& nb : neighbors) {
        if (nb.x >= 0 && nb.x < cols_ &&
            nb.y >= 0 && nb.y < rows_ &&
            nb.z >= 0 && nb.z < layers_ &&
            !obstacles_[idx(nb)]) {
            result.push_back(nb);
        }
    }
    return result;
}

std::vector<Vec3> DStarAlgorithm::getPredecessors(const Vec3& u) const {
    return getSuccessors(u);  // undirected grid
}

std::vector<Vec3> DStarAlgorithm::getAllNeighbors(const Vec3& u) const {
    std::vector<Vec3> result;
    const auto neighbors = getNeighbors(u, connectivity_);
    result.reserve(neighbors.size());

    for (const auto& nb : neighbors) {
        if (nb.x >= 0 && nb.x < cols_ &&
            nb.y >= 0 && nb.y < rows_ &&
            nb.z >= 0 && nb.z < layers_) {
            result.push_back(nb);
        }
    }
    return result;
}

double DStarAlgorithm::moveCost(const Vec3& a, const Vec3& b) const {
    const double base = calculateStepCost(a, b);  // 1.0, √2, or √3
    return base * (1.0 + get_cost_(b));           // terrain-weighted edge cost
}

// ===================================================================
// Helpers
// ===================================================================

void DStarAlgorithm::checkBounds(const Vec3& p) const {
    if (p.x < 0 || p.x >= cols_ ||
        p.y < 0 || p.y >= rows_ ||
        p.z < 0 || p.z >= layers_) {
        throw std::out_of_range(
            "Vec3 (" + std::to_string(p.x) + "," + std::to_string(p.y) +
            "," + std::to_string(p.z) + ") out of bounds [0," +
            std::to_string(cols_ - 1) + "]x[0," + std::to_string(rows_ - 1) +
            "]x[0," + std::to_string(layers_ - 1) + "]");
    }
}

}  // namespace astar
