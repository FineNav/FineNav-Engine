// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "occ_grid_map_view.hpp"
#include <cmath>

namespace bfs {

// ── OBB 硬编码参数（与 MPPI SimpleGridMapView 保持一致）──────────────────
// 尺寸 (L=0.8m, W=0.7m, H=0.3m)，表面采样步长 0.1m
static constexpr double kObbL    = 0.8;
static constexpr double kObbW    = 0.7;
static constexpr double kObbH    = 0.3;
static constexpr double kObbStep = 0.1;

// ── 构造 ──────────────────────────────────────────────────────────────────

OccGridMapView::OccGridMapView(const finenav::OccGrid2DMap& map,
                               const OccGridMapViewConfig&  view_config)
    : obb_model_(finenav::Vector3D(kObbL, kObbW, kObbH),
                 finenav::Pose::Identity(),
                 finenav::OBBCollisionPolicy{kObbStep})
{
    if (!map.hasMap()) {
        fprintf(stderr, "[bfs::OccGridMapView] WARNING: 地图尚无数据，所有格子不可通行！\n");
        return;
    }

    const auto& grid = map.grid();
    const auto& cfg  = map.config();

    resolution_          = grid.info.resolution;
    width_               = grid.info.width;
    height_              = grid.info.height;
    origin_x_            = grid.info.origin.position.x;
    origin_y_            = grid.info.origin.position.y;
    cost_scaling_factor_ = view_config.cost_scaling_factor;

    binarizeMap(grid, cfg.occupancy_threshold);
    buildCostMap(view_config.inflation_radius, view_config.cost_scaling_factor);

    // possibly_inscribed_cost：OBB 外径半径处的指数衰减代价
    // 中心点代价 >= 此值说明机器人某部分可能与障碍接触，需要 OBB 精确判断
    const double obb_outer_radius = obb_model_.getOuterDiameter() / 2.0;
    possibly_inscribed_cost_ = computeExpCost(static_cast<float>(obb_outer_radius));
}

// ── IMapView 接口 ─────────────────────────────────────────────────────────

bool OccGridMapView::isWalkable(const finenav::Position3D& p) const {
    int col, row;
    if (!worldToGrid(p.x(), p.y(), col, row)) return false;

    const uint8_t raw = cost_map_.at<uint8_t>(row, col);

    // 障碍格本身（cost=254）→ 直接不可通行
    if (raw == 254) return false;

    const int cost = static_cast<int>(raw);

    // 代价低于阈值 → 中心点距障碍足够远，直接可通行
    if (cost < possibly_inscribed_cost_) return true;

    // 代价 >= 阈值 → 机器人轮廓可能与障碍接触，用 OBB 精确判断
    // BFS 规划时姿态未知，保守地假设机器人朝向任意方向均对齐栅格（yaw=0）
    // 此处直接判断以中心点为原点、yaw=0 的 OBB 是否碰撞
    finenav::Pose pose = finenav::Pose::Identity();
    pose.translation() = finenav::Vector3D(p.x(), p.y(), 0.0);

    return !obb_model_.checkCollision(pose, [this](const finenav::Position3D& pt) -> bool {
        int c, r;
        if (!worldToGrid(pt.x(), pt.y(), c, r)) return false; // 超出地图 → 视为无碰撞
        const uint8_t v = cost_map_.at<uint8_t>(r, c);
        return v == 254; // 只有障碍格本身才触发碰撞
    });
}

double OccGridMapView::getStepSize() const {
    return resolution_;
}

// ── 私有：地图构建 ─────────────────────────────────────────────────────────

void OccGridMapView::binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold) {
    binary_map_ = cv::Mat(static_cast<int>(height_), static_cast<int>(width_), CV_8UC1);
    for (uint32_t row = 0; row < height_; ++row) {
        for (uint32_t col = 0; col < width_; ++col) {
            const int8_t val = msg.data[row * width_ + col];
            // unknown(-1) 保守地视为障碍
            binary_map_.at<uint8_t>(static_cast<int>(row), static_cast<int>(col)) =
                (val >= 0 && val < threshold) ? 0 : 254;
        }
    }
}

void OccGridMapView::buildCostMap(double inflation_radius, double cost_scaling_factor) {
    if (binary_map_.empty()) {
        fprintf(stderr, "[bfs::OccGridMapView] buildCostMap: binary_map_ 为空，跳过！\n");
        return;
    }

    // distanceTransform 输入：0=障碍（源），非0=自由
    // 反转后：障碍→0，自由→254
    cv::Mat inverted;
    cv::subtract(cv::Scalar(254), binary_map_, inverted);

    cv::Mat dist_px;
    cv::distanceTransform(inverted, dist_px, cv::DIST_L2, 5);

    const float res  = static_cast<float>(resolution_);
    const float infl = static_cast<float>(inflation_radius);

    cost_map_ = cv::Mat(dist_px.rows, dist_px.cols, CV_8UC1);
    for (int row = 0; row < dist_px.rows; ++row) {
        for (int col = 0; col < dist_px.cols; ++col) {
            const float dist_m = dist_px.at<float>(row, col) * res;

            uint8_t cost;
            if (dist_m == 0.0f) {
                // 障碍格本身
                cost = 254;
            } else if (dist_m > infl) {
                // 超出膨胀半径，代价归零
                cost = 0;
            } else {
                // 纯指数衰减，无硬膨胀区
                cost = static_cast<uint8_t>(
                    253.0 * std::exp(-cost_scaling_factor * static_cast<double>(dist_m)));
            }
            cost_map_.at<uint8_t>(row, col) = cost;
        }
    }
}

// ── 私有：坐标转换 ─────────────────────────────────────────────────────────

bool OccGridMapView::worldToGrid(double x, double y, int& col, int& row) const {
    if (cost_map_.empty()) return false;
    col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return col >= 0 && col < static_cast<int>(width_) &&
           row >= 0 && row < static_cast<int>(height_);
}

// ── 私有：代价计算辅助 ─────────────────────────────────────────────────────

int OccGridMapView::computeExpCost(float dist_m) const {
    return static_cast<int>(
        253.0 * std::exp(-cost_scaling_factor_ * static_cast<double>(dist_m)));
}

} // namespace bfs
