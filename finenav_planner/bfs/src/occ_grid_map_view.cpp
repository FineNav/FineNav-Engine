// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "occ_grid_map_view.hpp"
#include <cmath>

namespace bfs {

OccGridMapView::OccGridMapView(const finenav::OccGrid2DMap& map,
                               const OccGridMapViewConfig&  view_config) {
    if (!map.hasMap()) {
        fprintf(stderr, "[bfs::OccGridMapView] WARNING: map has no data yet, all cells will be non-walkable!\n");
        return;
    }

    const auto& grid = map.grid();
    const auto& cfg  = map.config();

    resolution_ = grid.info.resolution;
    width_      = grid.info.width;
    height_     = grid.info.height;
    origin_x_   = grid.info.origin.position.x;
    origin_y_   = grid.info.origin.position.y;

    binarizeMap(grid, cfg.occupancy_threshold);
    buildInflationMap(view_config.lethal_radius, view_config.inflation_radius);
}

// ── IMapView 接口 ──────────────────────────────────────────────────────────

bool OccGridMapView::isWalkable(const finenav::Position3D& p) const {
    int col, row;
    if (!worldToGrid(p.x(), p.y(), col, row)) return false;
    return walkable_map_.at<uint8_t>(row, col) > 0;
}

double OccGridMapView::getStepSize() const {
    return resolution_;
}

// ── 私有处理 ───────────────────────────────────────────────────────────────

void OccGridMapView::binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold) {
    binary_map_ = cv::Mat(static_cast<int>(height_), static_cast<int>(width_), CV_8UC1);
    for (uint32_t row = 0; row < height_; ++row) {
        for (uint32_t col = 0; col < width_; ++col) {
            const int8_t val = msg.data[row * width_ + col];
            // unknown(-1) 视为障碍，occupied(≥threshold) 视为障碍，free 视为0
            binary_map_.at<uint8_t>(static_cast<int>(row), static_cast<int>(col)) =
                (val >= 0 && val < threshold) ? 0 : 254;
        }
    }
}

void OccGridMapView::buildInflationMap(double lethal_radius, double inflation_radius) {
    if (binary_map_.empty()) {
        fprintf(stderr, "[bfs::OccGridMapView] buildInflationMap: binary_map_ is empty, skipping!\n");
        return;
    }

    // distanceTransform: 计算每个自由格到最近障碍格的像素距离
    // 输入: 0=障碍(源), 非0=自由
    cv::Mat inverted;
    cv::subtract(cv::Scalar(254), binary_map_, inverted); // 障碍→0，自由→254

    cv::Mat dist_px;
    cv::distanceTransform(inverted, dist_px, cv::DIST_L2, 5);

    // 生成可通行掩码：距障碍距离 > inflation_radius 才可通行
    const float res   = static_cast<float>(resolution_);
    const float infl  = static_cast<float>(inflation_radius);

    walkable_map_ = cv::Mat(dist_px.rows, dist_px.cols, CV_8UC1);
    for (int row = 0; row < dist_px.rows; ++row) {
        for (int col = 0; col < dist_px.cols; ++col) {
            const float dist_m = dist_px.at<float>(row, col) * res;
            walkable_map_.at<uint8_t>(row, col) = (dist_m > infl) ? 255 : 0;
        }
    }
}

bool OccGridMapView::worldToGrid(double x, double y, int& col, int& row) const {
    if (walkable_map_.empty()) return false;
    col = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    row = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return col >= 0 && col < static_cast<int>(width_) &&
           row >= 0 && row < static_cast<int>(height_);
}

} // namespace bfs
