// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "finenav_core/math_def.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_map_terrain_analysis/terrain_analysis_params.hpp"
#include "finenav_map_grid_map/grid_map.hpp"
#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"
#include "finenav_map_temporal_voxel_map/voxel.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav {

/**
 * @brief TerrainAnalyzer — ground detection, passability, and costmap generation.
 *
 * Processes a TemporalVoxelMap (3D dynamic voxel grid) together with a static
 * OccupancyGrid2D to produce:
 *   1. ground array (Eigen::ArrayXXf) — detected ground heights per column
 *   2. passability image (cv::Mat)  — binary: 0=free, 254=obstacle
 *   3. costmap image (cv::Mat)     — inflated: exponential-decay cost [0, 254]
 *
 * Designed to be called from MapServer's postUpdateHook:
 * @code
 *   map_server->registerPostUpdateHook(
 *       [&](const RobotState& s, TemporalVoxelMap& map) {
 *           terrain.update(s.position.z(), map, occ_map);
 *       });
 * @endcode
 *
 * Parameters are managed via AlgoConfigurator (generate_parameter_library YAML).
 * Visualisation uses FineNavVisualizer (optional — pass nullptr to disable).
 */
class TerrainAnalyzer {
public:
    using ConfigType = terrain_analysis::Params;

    TerrainAnalyzer() = default;

    /// Receive FineNavVisualizer injected by AlgoConfigurator.
    void setVisualizer(finenav::viz::FineNavVisualizer& viz) { viz_ = &viz; }
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    // ── Configuration ────────────────────────────────────────────────

    void configure(const ConfigType& cfg);

    // ── Main update cycle ────────────────────────────────────────────

    /**
     * @brief Run one full terrain-analysis + inflation pass.
     *
     * Must be called with the temporal map already write-locked (from postUpdateHook).
     *
     * @param robot_pose_z   Current robot z-height (for ground detection reference).
     * @param temporal_map   Temporal voxel map (read-only within this call).
     * @param occ_map        Static occupancy grid for obstacle fusion.
     */
    void update(double robot_pose_z,
                const TemporalVoxelMap& temporal_map,
                const OccupancyGrid2D& occ_map);

    /**
     * @brief Simplified update when no static map is available
     *        (occ_map is treated as empty — all cells free).
     */
    void update(double robot_pose_z, const TemporalVoxelMap& temporal_map);

    // ── Query ────────────────────────────────────────────────────────

    /// Query the inflated cost at a world-frame position.
    int getCost(const finenav::core::Position3D& pos, const GridMap<Voxel>& grid) const;

    /// Convert obstacle distance (cells) to cost via exponential decay.
    int computeCost(int distance_cells) const;

    /// Accessors for use by MapViews.
    double costScalingFactor() const { return cost_scaling_factor_; }
    bool   allowUnknown()        const { return allow_unknown_; }

private:
    // ── Pipeline stages ──────────────────────────────────────────────

    void terrainAnalysis(double robot_pose_z,
                         const GridMap<Voxel>& grid,
                         const OccupancyGrid2D* occ_map);
    void mapInflation();
    cv::Mat distanceTransform() const;
    void publishMaps(const GridMap<Voxel>& grid);

    static void applyGroundMeanFilter(Eigen::ArrayXXf& ground, int kernel_size);

    // ── Parameters ───────────────────────────────────────────────────

    double cost_scaling_factor_   = 0.5;
    double robot_height_          = 1.0;
    double max_gradient_          = 0.12;
    double nan_fill_radius_       = 1.0;
    double ground_z_offset_       = 0.125;
    bool   use_simple_z_threshold_ = false;
    double z_obstacle_threshold_  = 0.06;
    bool   allow_unknown_         = false;

    // ── Internal state ───────────────────────────────────────────────

    Eigen::ArrayXXf ground_array_;
    cv::Mat passability_image_;
    cv::Mat costmap_image_;
    double cached_resolution_ = 0.1;

    rclcpp::Logger logger_{rclcpp::get_logger("TerrainAnalyzer")};
    finenav::viz::FineNavVisualizer* viz_ = nullptr;
};

}  // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::TerrainAnalyzer, terrain_analysis)
