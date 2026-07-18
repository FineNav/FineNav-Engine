// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "stvl_manager.hpp"

using namespace finenav;

// ═══════════════════════════════════════════════════════════════════════════
// Angle utilities
// ═══════════════════════════════════════════════════════════════════════════

double StvlManager::normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ═══════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════

StvlManager::StvlManager(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
    node_->declare_parameter<double>("stvl_manager.decay_time_sec", 1000000.0);
    node_->declare_parameter<double>("stvl_manager.decay_time_fov_sec", 1000000.0);
    node_->declare_parameter<double>("stvl_manager.min_range", 0.0);
    node_->declare_parameter<double>("stvl_manager.max_range", 50.0);
    node_->declare_parameter<double>("stvl_manager.fov_start_angle", -M_PI_2);
    node_->declare_parameter<double>("stvl_manager.fov_end_angle",   M_PI_2);

    node_->get_parameter("stvl_manager.decay_time_sec", decay_time_sec_);
    node_->get_parameter("stvl_manager.decay_time_fov_sec", decay_time_fov_sec_);
    node_->get_parameter("stvl_manager.min_range", min_range_);
    node_->get_parameter("stvl_manager.max_range", max_range_);
    node_->get_parameter("stvl_manager.fov_start_angle", fov_start_angle_);
    node_->get_parameter("stvl_manager.fov_end_angle",   fov_end_angle_);
}

// ═══════════════════════════════════════════════════════════════════════════
// FOV
// ═══════════════════════════════════════════════════════════════════════════

bool StvlManager::isInFov(const Eigen::Isometry3d & T_map_body,
                          const Position & p_world) const
{
    const Eigen::Vector3d origin = T_map_body.translation();
    const Eigen::Vector3d v_world = p_world - origin;

    const double r = v_world.norm();
    if (r <= 0.0 || r < min_range_ || r > max_range_) {
        return false;
    }

    // Rotate to body frame, compute azimuth in XY plane.
    // 0 = +X (forward), +π/2 = +Y (left), -π/2 = -Y (right).
    const Eigen::Vector3d v_body = T_map_body.linear().transpose() * v_world;
    const double azi = std::atan2(v_body.y(), v_body.x());

    // Single [start, end] range with wraparound.
    // Normalise everything so we can write a clean interval check.
    double a   = normalizeAngle(azi);
    double s   = normalizeAngle(fov_start_angle_);
    double e   = normalizeAngle(fov_end_angle_);
    double mid = normalizeAngle(s + normalizeAngle(e - s) * 0.5);

    // Compute signed deviation from the sector centre and compare to half-width.
    double dev   = normalizeAngle(a - mid);
    double width = normalizeAngle(e - s);
    return std::abs(dev) <= std::abs(width) * 0.5 + 1e-9;
}

// ═══════════════════════════════════════════════════════════════════════════
// PruneExpiredCells
// ═══════════════════════════════════════════════════════════════════════════

void StvlManager::PruneExpiredCells(GridMap<Voxel> & map, const Eigen::Isometry3d & T_map_body)
{
    const rclcpp::Time now = node_->get_clock()->now();

    for (auto it = dynamic_cells_.begin(); it != dynamic_cells_.end();) {
        const Position & pos = *it;

        if (!map.isInside(pos)) {
            it = dynamic_cells_.erase(it);
            continue;
        }

        Voxel & cell = map.atPosition(pos);
        if (cell.timestamp.nanoseconds() == 0) {
            it = dynamic_cells_.erase(it);
            continue;
        }

        const double age_sec = (now - cell.timestamp).seconds();
        if (age_sec > decay_time_sec_) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
            continue;
        }

        const bool in_fov = isInFov(T_map_body, pos);
        const double active_decay = in_fov ? decay_time_fov_sec_ : decay_time_sec_;

        if (age_sec > active_decay) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
        } else {
            ++it;
        }
    }
}

void StvlManager::InsertPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, GridMap<Voxel> & map,
                                   const Eigen::Isometry3d & T_map_body) {
    const rclcpp::Time now = node_->get_clock()->now();
    for (const auto & pt : cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        // 点云已在车体坐标系下，直接用 T_map_body 变换到 map 系
        const Eigen::Vector3d p_world = T_map_body * Eigen::Vector3d(pt.x, pt.y, pt.z);
        const Position hit_pos(p_world.x(), p_world.y(), p_world.z());
        if (!map.isInside(hit_pos)) {
            continue;
        }
        const Index idx = map.getIndex(hit_pos);
        Voxel & cell = map.at(idx);
        if (!cell.is_static) {
            cell.z_height = static_cast<float>(p_world.z());
            cell.timestamp = now;
            dynamic_cells_.insert(map.getPosition(idx));
        }
    }
}