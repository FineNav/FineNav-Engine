// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_map_terrain_analysis/terrain_analyzer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace finenav {

using finenav::core::Position3D;
using finenav::core::Region3D;

// ==============================================================================
// Configuration
// ==============================================================================

void TerrainAnalyzer::configure(const ConfigType& cfg) {
    cost_scaling_factor_    = cfg.cost_scaling_factor;
    robot_height_           = cfg.robot_height;
    max_gradient_           = cfg.max_gradient;
    nan_fill_radius_        = cfg.nan_fill_radius;
    ground_z_offset_        = cfg.ground_z_offset;
    use_simple_z_threshold_ = cfg.use_simple_z_threshold;
    z_obstacle_threshold_   = cfg.z_obstacle_threshold;
    allow_unknown_          = cfg.allow_unknown;

    RCLCPP_INFO(logger_,
        "TerrainAnalyzer configured: cost_scale=%.2f, robot_h=%.2f m, "
        "max_grad=%.2f m, simple_z=%s",
        cost_scaling_factor_, robot_height_, max_gradient_,
        use_simple_z_threshold_ ? "true" : "false");
}

// ==============================================================================
// Public update (with occ_map)
// ==============================================================================

void TerrainAnalyzer::update(double robot_pose_z,
                             const TemporalVoxelMap& temporal_map,
                             const OccupancyGrid2D& occ_map) {
    const auto& grid = temporal_map.grid();
    cached_resolution_ = grid.getResolution();

    if (passability_image_.empty()) {
        passability_image_ = cv::Mat::zeros(grid.getSize().y(), grid.getSize().x(), CV_8UC1);
        costmap_image_     = cv::Mat::zeros(grid.getSize().y(), grid.getSize().x(), CV_8UC1);
    }

    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::duration<double, std::milli>;

    auto t0 = Clock::now();

    const OccupancyGrid2D* occ_ptr = occ_map.hasMap() ? &occ_map : nullptr;
    terrainAnalysis(robot_pose_z, grid, occ_ptr);

    auto t1 = Clock::now();
    mapInflation();
    auto t2 = Clock::now();
    publishMaps(grid);
    auto t3 = Clock::now();

    RCLCPP_DEBUG(logger_,
        "TerrainAnalyzer: terrain=%.1f ms, inflation=%.1f ms, publish=%.1f ms, total=%.1f ms",
        Ms(t1 - t0).count(), Ms(t2 - t1).count(),
        Ms(t3 - t2).count(), Ms(t3 - t0).count());
}

// ==============================================================================
// Public update (without occ_map)
// ==============================================================================

void TerrainAnalyzer::update(double robot_pose_z,
                             const TemporalVoxelMap& temporal_map) {
    const auto& grid = temporal_map.grid();
    cached_resolution_ = grid.getResolution();

    if (passability_image_.empty()) {
        passability_image_ = cv::Mat::zeros(grid.getSize().y(), grid.getSize().x(), CV_8UC1);
        costmap_image_     = cv::Mat::zeros(grid.getSize().y(), grid.getSize().x(), CV_8UC1);
    }

    terrainAnalysis(robot_pose_z, grid, nullptr);
    mapInflation();
    publishMaps(grid);
}

// ==============================================================================
// Query
// ==============================================================================

int TerrainAnalyzer::getCost(const Position3D& pos, const GridMap<Voxel>& grid) const {
    if (grid.isInside(pos)) {
        Index index = grid.getIndex(pos);
        return costmap_image_.at<uchar>(
            index.y() + grid.getSize().y() / 2,
            index.x() + grid.getSize().x() / 2);
    }
    return allow_unknown_ ? 0 : 254;
}

int TerrainAnalyzer::computeCost(int distance_cells) const {
    const double factor = std::exp(-1.0 * cost_scaling_factor_
                                   * (distance_cells * cached_resolution_));
    return static_cast<int>(253.0 * factor);
}

// ==============================================================================
// TerrainAnalysis (core algorithm)
// ==============================================================================

void TerrainAnalyzer::terrainAnalysis(double robot_pose_z,
                                       const GridMap<Voxel>& grid,
                                       const OccupancyGrid2D* occ_map) {
    const int sizeX_min = grid.getMinIndex().x();
    const int sizeX_max = grid.getMaxIndex().x();
    const int sizeY_min = grid.getMinIndex().y();
    const int sizeY_max = grid.getMaxIndex().y();
    const int size_x    = grid.getSize().x();
    const int size_y    = grid.getSize().y();
    const int half_x    = size_x / 2;
    const int half_y    = size_y / 2;

    ground_array_ = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

    std::vector<Voxel> voxels_along_z;
    voxels_along_z.reserve(grid.getSize().z());

    // ── Simple Z-threshold mode ──────────────────────────────────────────
    if (use_simple_z_threshold_) {
        for (int x = sizeX_min; x <= sizeX_max; ++x) {
            for (int y = sizeY_min; y <= sizeY_max; ++y) {
                grid.getVoxelsAlongZ(x, y, voxels_along_z);

                float max_z = -std::numeric_limits<float>::infinity();
                float min_z =  std::numeric_limits<float>::infinity();
                for (const auto& voxel : voxels_along_z) {
                    if (std::isfinite(voxel.z_height)) {
                        max_z = std::max(max_z, voxel.z_height);
                        min_z = std::min(min_z, voxel.z_height);
                    }
                }

                const int px = x + half_x;
                const int py = y + half_y;

                if (std::isfinite(max_z) && (max_z - robot_pose_z) > z_obstacle_threshold_) {
                    passability_image_.at<uchar>(py, px) = 254;
                } else {
                    passability_image_.at<uchar>(py, px) = 0;
                }
                ground_array_(px, py) = std::isfinite(min_z) ? min_z : NAN;
            }
        }
        applyGroundMeanFilter(ground_array_, 3);

        // Fill NaN near centre
        for (int x = 0; x < size_x; ++x) {
            for (int y = 0; y < size_y; ++y) {
                if (std::isnan(ground_array_(x, y))) {
                    double dist = std::hypot(x - half_x, y - half_y) * cached_resolution_;
                    if (dist < nan_fill_radius_) {
                        ground_array_(x, y) = static_cast<float>(robot_pose_z);
                    }
                }
            }
        }
        // Fuse global map and return early — done below
    } else {
        // ── Complex terrain analysis ─────────────────────────────────────
        std::vector<Voxel> valid_z_values;
        valid_z_values.reserve(grid.getSize().z());

        for (int x = sizeX_min; x <= sizeX_max; ++x) {
            for (int y = sizeY_min; y <= sizeY_max; ++y) {
                grid.getVoxelsAlongZ(x, y, voxels_along_z);

                valid_z_values.clear();
                for (const auto& voxel : voxels_along_z) {
                    if (std::isfinite(voxel.z_height)) {
                        valid_z_values.push_back(voxel);
                    }
                }

                float nearest_ground = std::numeric_limits<float>::max();
                if (!valid_z_values.empty()) {
                    for (size_t i = 0; i < valid_z_values.size() - 1; ++i) {
                        if (valid_z_values[i + 1].z_height -
                            valid_z_values[i].z_height > robot_height_) {
                            if (std::fabs(valid_z_values[i].z_height - robot_pose_z) <
                                std::fabs(nearest_ground - robot_pose_z)) {
                                nearest_ground = valid_z_values[i].z_height;
                            }
                        }
                    }
                    if (std::fabs(valid_z_values.back().z_height - robot_pose_z) <
                        std::fabs(nearest_ground - robot_pose_z)) {
                        nearest_ground = valid_z_values.back().z_height;
                    }
                } else {
                    nearest_ground = NAN;
                }
                ground_array_(x + half_x, y + half_y) = nearest_ground;
            }
        }

        applyGroundMeanFilter(ground_array_, 3);

        // Fill NaN near centre
        for (int x = 0; x < size_x; ++x) {
            for (int y = 0; y < size_y; ++y) {
                if (std::isnan(ground_array_(x, y))) {
                    double dist = std::hypot(x - half_x, y - half_y) * cached_resolution_;
                    if (dist < nan_fill_radius_) {
                        ground_array_(x, y) = static_cast<float>(robot_pose_z);
                    }
                }
            }
        }

        // Gradient-based passability
        for (int x = 0; x < size_x; ++x) {
            for (int y = 0; y < size_y; ++y) {
                const int dx[4] = {0, 1, 0, -1};
                const int dy[4] = {1, 0, -1, 0};

                uchar terrain_value = 0;
                float current_ground = ground_array_(x, y);
                if (std::isfinite(current_ground)) {
                    for (int i = 0; i < 4; ++i) {
                        int nx = x + dx[i];
                        int ny = y + dy[i];
                        if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y) continue;
                        float neighbor = ground_array_(nx, ny);
                        if (std::isfinite(neighbor) &&
                            std::fabs(current_ground - neighbor) > max_gradient_) {
                            terrain_value = 254;
                            break;
                        }
                    }
                }
                passability_image_.at<uchar>(y, x) = terrain_value;
            }
        }
    }

    // ── Fuse global occupancy grid (common to both modes) ──────────────
    if (occ_map) {
        const auto& info = occ_map->grid().info;
        const double map_res   = info.resolution;
        const double origin_x  = info.origin.position.x;
        const double origin_y  = info.origin.position.y;
        const int    map_w     = info.width;
        const int    map_h     = info.height;

        for (int x = 0; x < size_x; ++x) {
            for (int y = 0; y < size_y; ++y) {
                Index idx(x - half_x, y - half_y, 0);
                if (!grid.isInside(idx)) continue;
                auto pos = grid.getPosition(idx);
                int gx = static_cast<int>(std::floor((pos.x() - origin_x) / map_res));
                int gy = static_cast<int>(std::floor((pos.y() - origin_y) / map_res));
                if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h) {
                    if (occ_map->grid().data[gy * map_w + gx] >=
                        occ_map->config().occupancy_threshold) {
                        passability_image_.at<uchar>(y, x) = 254;
                    }
                }
            }
        }
    }
}

// ==============================================================================
// Ground mean filter
// ==============================================================================

void TerrainAnalyzer::applyGroundMeanFilter(Eigen::ArrayXXf& ground, int kernel_size) {
    int rows = ground.rows();
    int cols = ground.cols();
    Eigen::ArrayXXf filtered = ground;
    int half_k = kernel_size / 2;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (std::isnan(ground(r, c))) continue;

            float sum = 0.0f;
            int count = 0;
            for (int kr = -half_k; kr <= half_k; ++kr) {
                for (int kc = -half_k; kc <= half_k; ++kc) {
                    int nr = r + kr, nc = c + kc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                        float val = ground(nr, nc);
                        if (std::isfinite(val)) { sum += val; count++; }
                    }
                }
            }
            if (count > 0) filtered(r, c) = sum / count;
        }
    }
    ground = filtered;
}

// ==============================================================================
// MapInflation
// ==============================================================================

void TerrainAnalyzer::mapInflation() {
    cv::Mat dt = distanceTransform();
    for (int x = 0; x < passability_image_.cols; ++x) {
        for (int y = 0; y < passability_image_.rows; ++y) {
            if (passability_image_.at<uchar>(y, x) == 0) {
                float distance = dt.at<float>(y, x);
                costmap_image_.at<uchar>(y, x) =
                    static_cast<uchar>(computeCost(static_cast<int>(distance)));
            } else {
                costmap_image_.at<uchar>(y, x) = 254;
            }
        }
    }
}

cv::Mat TerrainAnalyzer::distanceTransform() const {
    cv::Mat binary = 254 - passability_image_;
    cv::Mat result;
    cv::distanceTransform(binary, result, cv::DIST_L2, 3);
    return result;
}

// ==============================================================================
// Visualisation
// ==============================================================================

void TerrainAnalyzer::publishMaps(const GridMap<Voxel>& grid) {
    if (!viz_) return;

    const int size_x = grid.getSize().x();
    const int size_y = grid.getSize().y();
    const int half_x = size_x / 2;
    const int half_y = size_y / 2;

    // Passability map: green=free, red=obstacle, grey=other
    if (viz_->pointCloud().hasSubscribers(viz_->cloudTopic())) {
        auto& cloud = viz_->pointCloud();
        cloud.setBaseFrame("map");
        cloud.reserve(size_x * size_y);
        for (int px = 0; px < size_x; ++px) {
            for (int py = 0; py < size_y; ++py) {
                Index idx(px - half_x, py - half_y, 0);
                if (!grid.isInside(idx)) continue;
                auto pos = grid.getPosition(idx);
                uint8_t val = passability_image_.at<uint8_t>(py, px);
                if (val == 0) {
                    cloud.addPoint(static_cast<float>(pos.x()),
                                   static_cast<float>(pos.y()), 0.0f, 0, 200, 0);
                } else if (val >= 254) {
                    cloud.addPoint(static_cast<float>(pos.x()),
                                   static_cast<float>(pos.y()), 0.0f, 200, 0, 0);
                } else {
                    cloud.addPoint(static_cast<float>(pos.x()),
                                   static_cast<float>(pos.y()), 0.0f, 120, 120, 120);
                }
            }
        }
        cloud.publish(viz_->cloudTopic() + "/passability");
    }

    // Ground height: jet colormap on separate cloud channel
    if (ground_array_.size() > 0) {
        auto& cloud = viz_->pointCloud();
        cloud.setBaseFrame("map");
        cloud.reserve(size_x * size_y);

        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        for (int px = 0; px < size_x; ++px)
            for (int py = 0; py < size_y; ++py) {
                float z = ground_array_(px, py);
                if (std::isfinite(z)) { z_min = std::min(z_min, z); z_max = std::max(z_max, z); }
            }
        float z_range = (z_max > z_min) ? (z_max - z_min) : 1.0f;

        for (int px = 0; px < size_x; ++px) {
            for (int py = 0; py < size_y; ++py) {
                float z = ground_array_(px, py);
                if (!std::isfinite(z)) continue;
                Index idx(px - half_x, py - half_y, 0);
                if (!grid.isInside(idx)) continue;
                auto pos = grid.getPosition(idx);
                float t = std::clamp((z - z_min) / z_range, 0.0f, 1.0f);
                uint8_t r = static_cast<uint8_t>(255 * std::clamp(1.5f - std::abs(4.0f * t - 3.0f), 0.0f, 1.0f));
                uint8_t g = static_cast<uint8_t>(255 * std::clamp(1.5f - std::abs(4.0f * t - 2.0f), 0.0f, 1.0f));
                uint8_t b = static_cast<uint8_t>(255 * std::clamp(1.5f - std::abs(4.0f * t - 1.0f), 0.0f, 1.0f));
                cloud.addPoint(static_cast<float>(pos.x()),
                               static_cast<float>(pos.y()), z, r, g, b);
            }
        }
        cloud.publish(viz_->cloudTopic() + "/ground");
    }
}

}  // namespace finenav
