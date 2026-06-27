// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Lightweight stub IMapView for MPPI benchmarking.
// Dense grid sampling for costAtPose / isCollision — matches OBB load (~100 pts).

#pragma once

#include "finenav_mppi_controller/map_view_def.hpp"

#include <array>
#include <cmath>
#include <random>

namespace bench {

enum class Scenario {
    kOpen,              // free space — absolute baseline
    kWallAhead,         // lethal wall 2m ahead — half-obstructed
    kInflationField,    // 1.5m alley with inflation gradient on walls
    kMinefield,         // ultra-dense pillars, narrow carved corridor
    kInflationGrid,     // sparse pillars @ 3m, wide inflation → heavy footprint cost
    kObbStress,         // ultra-dense pillars @ 0.6m + full-map inflation → OBB every frame
};

/// Lightweight grid-based IMapView with OBBCollisionModel for realistic overhead.
class BenchMapView : public nav2_mppi_controller::IMapView {
public:
    static constexpr int   kMapSize  = 600;
    static constexpr int   kLethal   = 254;
    static constexpr int   kInscribed= 253;
    static constexpr int   kNoInfo   = 255;
    static constexpr int   kFree     = 0;

    BenchMapView(Scenario scenario, double resolution = 0.05)
        : resolution_(resolution), map_size_(kMapSize), radius_(0.5f)
    {
        grid_.fill(kFree);
        buildScenario(scenario);
    }

    bool isTrackingUnknown() const override { return false; }
    bool considerFootprint()   const override { return footprint_; }
    void setFootprint(bool v)  { footprint_ = v; }

    bool isCollision(float x, float y, float theta) const override {
        return sampleFootprint(x, y, theta, /*early_exit=*/true) == kLethal;
    }

    int getCost(const finenav::Position3D& pos) const override {
        return cellCost(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
    }

    float getRadius() const override { return radius_; }

    float costAtPose(float x, float y, float theta) const override {
        return static_cast<float>(sampleFootprint(x, y, theta, /*early_exit=*/false));
    }

    std::string getBaseFrameID() const override { return "map"; }
    float resolution() const { return resolution_; }

private:
    // ── coordinate helpers ──
    int cellCost(float wx, float wy) const {
        if (!std::isfinite(wx) || !std::isfinite(wy)) return kNoInfo;
        int cx = static_cast<int>(wx / resolution_) + map_size_ / 2;
        int cy = static_cast<int>(wy / resolution_) + map_size_ / 2;
        if (cx < 0 || cy < 0 || cx >= map_size_ || cy >= map_size_) return kNoInfo;
        size_t idx = static_cast<size_t>(cy) * map_size_ + static_cast<size_t>(cx);
        if (idx >= kMapSize * kMapSize) return kNoInfo;
        return grid_[idx];
    }

    void setCost(float wx, float wy, int cost) {
        int cx = static_cast<int>(wx / resolution_) + map_size_ / 2;
        int cy = static_cast<int>(wy / resolution_) + map_size_ / 2;
        if (cx < 0 || cy < 0 || cx >= map_size_ || cy >= map_size_) return;
        if (static_cast<int>(grid_[cy * map_size_ + cx]) < cost)
            grid_[cy * map_size_ + cx] = static_cast<uint8_t>(cost);
    }

    // ── Dense grid footprint sampling — equivalent load to OBB edge check (~100 pts) ──
    int sampleFootprint(float x, float y, float theta, bool early_exit) const {
        float c = std::cos(theta), s = std::sin(theta);
        int max_cost = 0;
        // 0.8m × 0.7m footprint, 0.05m step → ~17×15 = 255 cells
        // Match OBB's ~96 edge points + inflation lookup overhead
        for (float dx = -0.4f; dx <= 0.4f; dx += 0.08f) {
            for (float dy = -0.35f; dy <= 0.35f; dy += 0.07f) {
                float wx = x + c * dx - s * dy;
                float wy = y + s * dx + c * dy;
                int cost = cellCost(wx, wy);
                if (early_exit && cost == kLethal) return kLethal;
                if (cost > max_cost) max_cost = cost;
            }
        }
        return max_cost;
    }

    // Simplified inflation decay for scenario building
    int computeCost(int distance_cells) const {
        if (distance_cells <= 0) return kLethal;
        if (distance_cells >= 10) return 0;
        return static_cast<int>(253.0 * (1.0 - static_cast<double>(distance_cells) / 10.0));
    }

    // ── drawing primitives ──
    void fillRect(float x0, float y0, float x1, float y1, int cost) {
        for (float x = x0; x <= x1; x += resolution_)
            for (float y = y0; y <= y1; y += resolution_)
                setCost(x, y, cost);
    }

    void fillCircle(float cx, float cy, float r, int core_cost) {
        for (float x = cx - r; x <= cx + r; x += resolution_)
            for (float y = cy - r; y <= cy + r; y += resolution_)
                if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r)
                    setCost(x, y, core_cost);
        // radial inflation gradient — match Nav2 costmap ~0.5m
        float max_r = r + 0.5f;
        for (float x = cx - max_r; x <= cx + max_r; x += resolution_)
            for (float y = cy - max_r; y <= cy + max_r; y += resolution_) {
                float d = std::sqrt((x - cx)*(x - cx) + (y - cy)*(y - cy));
                if (d > r && d <= max_r) {
                    int dc = static_cast<int>((d - r) / resolution_);
                    int c = computeCost(dc);
                    setCost(x, y, c);
                }
            }
    }

    void buildScenario(Scenario s) {
        switch (s) {
        case Scenario::kOpen:
            break;

        case Scenario::kWallAhead:
            fillRect(2.0f, 0.2f, 2.1f, 10.f, kLethal);
            for (int r = 1; r <= 20; ++r) {
                float x_infl = 2.0f - r * resolution_;
                for (float y = 0.2f; y <= 10.f; y += resolution_)
                    setCost(x_infl, y, computeCost(r));
            }
            break;

        case Scenario::kInflationField: {
            // 1.5m alley — robot (OBB 0.8×0.7m) fits with margin but
            // noise pushes many trajectory steps into inflation → heavy OBB.
            fillRect(-15.f, -15.f, 15.f, -0.75f, kLethal);   // bottom wall
            fillRect(-15.f,  0.75f, 15.f, 15.f,  kLethal);    // top wall
            // Inflation gradient into the alley from both sides
            for (int r = 1; r <= 10; ++r) {
                int c = computeCost(r);
                float off = r * resolution_;
                for (float x = -15.f; x <= 15.f; x += resolution_) {
                    setCost(x, -0.75f + off, c);   // bottom → up
                    setCost(x,  0.75f - off, c);   // top → down
                }
            }
            break;
        }

        case Scenario::kInflationGrid: {
            // Sparse pillars @ 3m with inflation radius 1.5m.
            // Fill every cell within inflation zone with distance-decayed cost.
            // Robot at (0,0) heading east: trajectory passes through cost 10-200 zones.
            constexpr float infl_radius = 1.0f;
            std::vector<std::pair<float,float>> pillars;
            for (float cx = -10.f; cx <= 10.f; cx += 1.5f)
                for (float cy = -8.f; cy <= 8.f; cy += 1.5f) {
                    // no gap — dense grid, robot at (0,0) heading east survives by noise
                    pillars.emplace_back(cx, cy);
                    fillCircle(cx, cy, 0.15f, kLethal);
                }
            // Fill inflation: cost = 253 * exp(-scaling * dist).
            const float scaling = 4.0f;  // exp(-4*1.0) ≈ 0.018 → 253*0.018 ≈ 5 at 1m edge
            int half = map_size_ / 2;
            for (int ix = 0; ix < map_size_; ++ix) {
                for (int iy = 0; iy < map_size_; ++iy) {
                    if (grid_[iy * map_size_ + ix] == kLethal) continue;
                    float wx = (ix - half) * resolution_;
                    float wy = (iy - half) * resolution_;
                    float min_dist = infl_radius;
                    for (auto& [px, py] : pillars) {
                        float d = std::hypot(wx - px, wy - py) - 0.2f;  // distance from obstacle edge
                        if (d < min_dist) min_dist = d;
                    }
                    if (min_dist <= 0.f) continue;
                    if (min_dist < infl_radius) {
                        int c = static_cast<int>(253.f * std::exp(-scaling * min_dist));
                        if (c > 0) grid_[iy * map_size_ + ix] = static_cast<uint8_t>(c);
                    }
                }
            }
            break;
        }

        case Scenario::kMinefield: {
            std::mt19937 rng(99);
            std::uniform_real_distribution<float> jx(-0.2f, 0.2f);
            std::uniform_real_distribution<float> jy(-0.2f, 0.2f);
            for (float cx = -12.f; cx <= 12.f; cx += 1.f) {
                for (float cy = -12.f; cy <= 12.f; cy += 1.f) {
                    fillCircle(cx + jx(rng), cy + jy(rng), 0.18f, kLethal);
                }
            }
            for (float x = -10.f; x <= 5.f; x += resolution_)
                for (float y = -0.75f; y <= 0.75f; y += resolution_)
                    setCost(x, y, kFree);
            break;
        }

        case Scenario::kObbStress: {
            // Pillars every 0.6 m, radius 0.15 m → lethal cores cover ±6 m
            for (float cx = -6.f; cx <= 6.f; cx += 0.6f)
                for (float cy = -6.f; cy <= 6.f; cy += 0.6f)
                    fillCircle(cx, cy, 0.15f, kLethal);

            // Full-map inflation: cost = 253 * exp(-scaling * dist)
            // scaling=4.0 → exp(-4*1.0)≈0.018, cost at 1 m edge ≈ 5
            constexpr float infl_radius = 1.0f;
            constexpr float scaling     = 4.0f;
            int half = map_size_ / 2;
            for (int ix = 0; ix < map_size_; ++ix) {
                for (int iy = 0; iy < map_size_; ++iy) {
                    if (grid_[iy * map_size_ + ix] == kLethal) continue;
                    float wx = (ix - half) * resolution_;
                    float wy = (iy - half) * resolution_;
                    // Find nearest pillar
                    float min_dist = infl_radius;
                    for (float cx = -6.f; cx <= 6.f; cx += 0.6f) {
                        for (float cy = -6.f; cy <= 6.f; cy += 0.6f) {
                            float d = std::hypot(wx - cx, wy - cy) - 0.15f; // from edge
                            if (d < min_dist) min_dist = d;
                        }
                    }
                    if (min_dist <= 0.f) continue;
                    if (min_dist < infl_radius) {
                        int c = static_cast<int>(253.f * std::exp(-scaling * min_dist));
                        if (c > 0) grid_[iy * map_size_ + ix] = static_cast<uint8_t>(c);
                    }
                }
            }
            break;
        }
        }
    }

    double  resolution_;
    int     map_size_;
    float   radius_;
    bool    footprint_{true};

    std::array<uint8_t, kMapSize * kMapSize> grid_{};
};

}  // namespace bench
