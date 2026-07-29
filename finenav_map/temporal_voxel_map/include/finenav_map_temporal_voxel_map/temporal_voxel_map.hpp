// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"
#include "finenav_map_grid_map/grid_map.hpp"
#include "finenav_map_temporal_voxel_map/temporal_voxel_map_params.hpp"
#include "finenav_map_temporal_voxel_map/voxel.hpp"

namespace finenav {

using finenav::core::Position3D;
using finenav::core::Region3D;

/**
 * @brief TemporalVoxelMap — 3D voxel grid with time-based dynamic obstacle filtering.
 *
 * Wraps GridMap<Voxel> internally and adds:
 *   - Point-cloud insertion with single-cone FOV filtering
 *   - Time-based pruning of expired dynamic voxels
 *
 * Satisfies SpatialContainer and ShiftableMap — usable with MapServer.
 * All methods are called from the MapServer update thread (write lock held),
 * so no additional locking is needed.
 */
class TemporalVoxelMap {
public:
    using ConfigType = temporal_voxel_map::Params;
    using DataType   = Voxel;

    TemporalVoxelMap() = default;

    /// Receive FineNavVisualizer injected by AlgoConfigurator.
    void setVisualizer(finenav::viz::FineNavVisualizer& viz) { viz_ = &viz; }
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    // ── MapConcept ─────────────────────────────────────────────────

    void configure(const ConfigType& cfg);

    bool isInside(const Position3D& pos) const { return grid_.isInside(pos); }

    Region3D getWindowBounds() const { return grid_.getWindowBounds(); }

    Position3D getWindowCenter() const { return grid_.getWindowCenter(); }

    void shiftWindowTo(const Position3D& pos) { grid_.shiftWindowTo(pos); }

    // ── Sensor data injection ─────────────────────────────────────

    /**
     * @brief Transform and insert a point cloud into the voxel grid.
     *
     * Points outside the single-cone FOV or outside the map bounds are skipped.
     * Each inserted voxel's timestamp is set to steady_clock::now() and its
     * position is added to the dynamic-cell set for later pruning.
     *
     * @param cloud       Point cloud in body frame.
     * @param T_map_body  Body-to-map transform.
     */
    void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                          const Eigen::Isometry3d& T_map_body);

    // ── Temporal decay ────────────────────────────────────────────

    /**
     * @brief Remove voxels whose timestamp exceeds the configured lifetime.
     *
     * Called from MapServer's preUpdateHook.  Voxels inside the FOV use
     * decay_time_fov_sec_; those outside use decay_time_sec_.
     *
     * @param T_map_body  Current body-to-map transform (for FOV check).
     */
    void pruneExpiredCells(const Eigen::Isometry3d& T_map_body);

    // ── Public query API (for MapView adapters) ──────────────────

    /// Read-only access to the underlying 3D voxel grid.
    /// Needed by algorithms (TerrainAnalyzer) that iterate over voxel columns.
    const GridMap<Voxel>& grid() const { return grid_; }

    /**
     * @brief Check whether a world-coordinate position has an active
     *        (non-expired, non-static) dynamic obstacle voxel.
     *
     * Used by composite MapViews that fuse static occupancy grids with
     * temporal dynamic-obstacle data for local planning (MPPI).
     *
     * @param pos  World-frame query position (exact 3D match).
     * @return true if an active dynamic obstacle voxel exists at pos.
     */
    bool isDynamicObstacle(const Position3D& pos) const;

    /**
     * @brief Check whether any voxel in the (x, y) column within [z_min, z_max]
     *        contains an active dynamic obstacle.
     *
     * Designed for 2D planners (MPPI) that query in the ground plane —
     * this sweeps the robot-height range to catch obstacles at any elevation.
     *
     * @param x, y  World-frame horizontal coordinates.
     * @param z_min Lower bound of the vertical sweep (m).
     * @param z_max Upper bound of the vertical sweep (m).
     * @return true if ANY voxel in the column is a dynamic obstacle.
     */
    bool isDynamicObstacleInColumn(double x, double y,
                                   double z_min = 0.0,
                                   double z_max = 2.0) const;

private:
    /// Single-cone FOV: polar angle from body +Z axis.
    bool isInFrustum(const Position& p_world,
                     const Eigen::Isometry3d& T_map_body) const;

    static double normalizeAngle(double angle);

    // ── Internal state ────────────────────────────────────────────

    GridMap<Voxel> grid_;
    DynamicPosSet   dynamic_cells_;

    // ── Parameters (set by configure) ─────────────────────────────

    double min_range_          = 0.0;
    double max_range_          = 50.0;
    double min_angle_rad_      = 0.0;
    double max_angle_rad_      = 1.047;   // π/3
    double decay_time_sec_     = 10.0;
    double decay_time_fov_sec_ = 1.0;

    rclcpp::Logger                logger_{rclcpp::get_logger("TemporalVoxelMap")};
    finenav::viz::FineNavVisualizer* viz_ = nullptr;
};

}  // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::TemporalVoxelMap, temporal_voxel_map)
