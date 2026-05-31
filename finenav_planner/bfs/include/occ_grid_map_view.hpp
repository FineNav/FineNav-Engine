// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "bfs_imap_view.hpp"
#include "occ_grid_2d_map.hpp"

namespace bfs {

/**
 * @brief OccGridMapView 的处理参数
 *
 * 描述如何将原始占用值解释为可通行性，与地图数据本身无关。
 * BFS 只需要二值化结果，不需要代价值。
 */
struct OccGridMapViewConfig {
    double lethal_radius    = 0.20; ///< 硬膨胀半径 (m)，范围内视为不可通行
    double inflation_radius = 0.50; ///< 软膨胀半径 (m)，范围内也视为不可通行（保守策略）
};

/**
 * @brief OccGridMapView — 基于 OccGrid2DMap 的 bfs::IMapView 实现
 *
 * BFS 使用均等代价搜索，只需区分可通行/不可通行，无需 getCost()。
 * 在构造时读取地图数据并完成二值化 + 膨胀，之后只读。
 */
class OccGridMapView : public IMapView {
public:
    explicit OccGridMapView(const finenav::OccGrid2DMap& map,
                            const OccGridMapViewConfig&  view_config = {});

    // --- IMapView 接口 ---
    bool   isWalkable(const finenav::Position3D& p) const override;
    double getStepSize()                            const override;

private:
    void binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold);
    void buildInflationMap(double lethal_radius, double inflation_radius);
    bool worldToGrid(double x, double y, int& col, int& row) const;

    double   resolution_ = 0.05;
    uint32_t width_      = 0;
    uint32_t height_     = 0;
    double   origin_x_   = 0.0;
    double   origin_y_   = 0.0;

    cv::Mat binary_map_;     ///< 原始二值地图：254=障碍/未知，0=自由
    cv::Mat walkable_map_;   ///< 膨胀后可通行掩码：0=不可通行，255=可通行
};

} // namespace bfs
