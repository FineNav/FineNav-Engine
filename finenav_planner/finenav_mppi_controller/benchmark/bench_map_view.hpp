// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Realistic IMapView for MPPI benchmarking.
// Uses OBBCollisionModel (same as production) over a 2D inflated grid.

#pragma once

#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"

#include <array>
#include <cmath>
#include <Eigen/Geometry>
#include <random>

namespace bench {

enum class Scenario {
    kOpen,              // free space — absolute baseline
    kWallAhead,         // lethal wall 2m ahead — half-obstructed
    kInflationField,    // 1.5m alley with inflation gradient on walls
    kMinefield,         // ultra-dense pillars, narrow carved corridor
    kInflationGrid,     // sparse pillars @ 3m, wide inflation → heavy footprint cost
    kObbStress,         // ultra-dense pillars @ 0.6m + full-map inflation → OBB every frame
    kCorridorObstacle,  // 2m corridor + obstacle @ 1m →绕行，每点触发OBB, 无提前退出
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
        : resolution_(resolution), map_size_(kMapSize)
    {
        grid_.fill(kFree);
        buildScenario(scenario);
        // Outer diameter from production OBB (0.8×0.7×0.3): sqrt(0.8²+0.7²+0.3²)/2 ≈ 0.55
        radius_ = 0.55f;
    }

    bool isTrackingUnknown() const override { return false; }
    bool considerFootprint()   const override { return footprint_; }
    void setFootprint(bool v)  { footprint_ = v; }

    bool isCollision(float x, float y, float theta) const override {
        // Mirror SimpleGridMapView::isCollision: center-cost threshold gates OBB check
        int pose_cost = getCost(finenav::Position3D{x, y, 0.0});
        if (pose_cost == kLethal) return true;
        auto& obb = getThreadObb();
        int possible_collision_cost = computeCost(
            static_cast<int>(obb.getOuterDiameter() / 2.0 / resolution_));
        if (pose_cost >= possible_collision_cost) {
            finenav::Pose pose = createPose2D(x, y, theta);
            return obb.checkCollisionEdge(pose, [this](const finenav::Position3D& pos) -> bool {
                return cellCost(static_cast<float>(pos.x()), static_cast<float>(pos.y())) == kLethal;
            });
        }
        return false;
    }

    int getCost(const finenav::Position3D& pos) const override {
        return cellCost(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
    }

    float getRadius() const override { return radius_; }

    float costAtPose(float x, float y, float theta) const override {
        // Mirror SimpleGridMapView::costAtPose: full OBB edge cost
        finenav::Pose pose = createPose2D(x, y, theta);
        auto& obb = getThreadObb();
        return static_cast<float>(obb.checkCostEdge(pose, [this](const finenav::Position3D& pos) -> int {
            return cellCost(static_cast<float>(pos.x()), static_cast<float>(pos.y()));
        }));
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

    finenav::Pose createPose2D(double x, double y, double theta) const {
        finenav::Pose pose = finenav::Pose::Identity();
        pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        pose.translation() = Eigen::Vector3d(x, y, 0.0);
        return pose;
    }

    // Inflation decay matching terrain_analysis::ComputeCost:
    //   cost = 253 * exp(-cost_scaling_factor * distance_meters)
    // with cost_scaling_factor = 0.5 (same as finenav_params.yaml default).
    int computeCost(int distance_cells) const {
        if (distance_cells <= 0) return kLethal;
        const double dist_m = static_cast<double>(distance_cells) * resolution_;
        int c = static_cast<int>(253.0 * std::exp(-0.5 * dist_m));
        return c > 0 ? c : 0;
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

            // Carve a clear starting zone around (0,0) extending east,
            // so trajectories must travel through inflation before hitting pillars.
            // Robot at (0,0) heading east: first ~0.5 m is free, then enters dense cost field.
            for (float x = -0.3f; x <= 0.8f; x += resolution_)
                for (float y = -0.5f; y <= 0.5f; y += resolution_)
                    setCost(x, y, kFree);

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

        case Scenario::kCorridorObstacle: {
            // 2 m wide corridor (walls at y=±1.0), round obstacle at (1.0, 0) r=0.25.
            // Robot at (0,0) heading east, 1.5 m prediction reach → must steer around.
            // Inflation via terrain formula 253*exp(-0.5*d) fills the corridor → every
            // trajectory point triggers isCollision, non-colliding trajectories run full length.
            fillRect(-15.f,  1.0f, 15.f, 15.f,  kLethal);   // top wall
            fillRect(-15.f, -1.0f, 15.f, -15.f, kLethal);   // bottom wall
            fillCircle(1.0f, 0.0f, 0.25f, kLethal);          // obstacle @ 1m ahead

            // Inflation from walls and obstacle into corridor, terrain formula
            constexpr float infl_radius = 2.0f;
            int half = map_size_ / 2;
            for (int ix = 0; ix < map_size_; ++ix) {
                for (int iy = 0; iy < map_size_; ++iy) {
                    if (grid_[iy * map_size_ + ix] == kLethal) continue;
                    float wx = (ix - half) * resolution_;
                    float wy = (iy - half) * resolution_;
                    // Distance to top wall, bottom wall, obstacle
                    float d_top  = wy - 1.0f;
                    float d_bot  = -1.0f - wy;
                    float d_obs  = std::hypot(wx - 1.0f, wy) - 0.25f;
                    float min_dist = std::min({d_top, d_bot, d_obs, infl_radius});
                    if (min_dist <= 0.f) continue;
                    if (min_dist < infl_radius) {
                        int dc = static_cast<int>(min_dist / resolution_);
                        int c  = computeCost(dc);
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

    /// Thread-local OBB: each thread gets its own instance → zero contention.
    static finenav::OBBCollisionModel& getThreadObb() {
        thread_local auto obb = std::make_unique<finenav::OBBCollisionModel>(
            finenav::Vector3D(0.8, 0.7, 0.3),
            finenav::Pose::Identity(),
            finenav::OBBCollisionPolicy{0.1});
        return *obb;
    }

    std::array<uint8_t, kMapSize * kMapSize> grid_{};
};

}  // namespace bench
