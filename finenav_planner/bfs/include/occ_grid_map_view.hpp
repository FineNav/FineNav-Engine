// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "bfs_imap_view.hpp"
#include "occ_grid_2d_map.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"

namespace bfs {

/**
 * @brief OccGridMapView 的处理参数
 *
 * 膨胀策略：纯指数衰减，无硬膨胀区。
 * cost = 253 * exp(-cost_scaling_factor * dist_m)
 * 当中心点代价 >= possibly_inscribed_cost 时，触发 OBB 精确碰撞检查。
 */
struct OccGridMapViewConfig {
    double inflation_radius    = 0.50; ///< 膨胀半径 (m)，超出此范围代价归零
    double cost_scaling_factor = 10.0; ///< 指数衰减系数，越大衰减越快
};

/**
 * @brief OccGridMapView — 基于 OccGrid2DMap 的 bfs::IMapView 实现
 *
 * 膨胀逻辑参照 MPPI CostCritic：
 *   1. 对每格计算纯指数衰减代价（无硬膨胀区）
 *   2. 中心点代价 < possibly_inscribed_cost_ → 可通行
 *   3. 中心点代价 >= possibly_inscribed_cost_ → 用 OBBCollisionModel 精确判断
 *
 * OBB 尺寸硬编码为 (0.8, 0.7, 0.3)，与 MPPI SimpleGridMapView 一致。
 * unknown(-1) 视为障碍（保守策略）。
 */
class OccGridMapView : public IMapView {
public:
    explicit OccGridMapView(const finenav::OccGrid2DMap& map,
                            const OccGridMapViewConfig&  view_config = {});

    // ── IMapView 接口 ──────────────────────────────────────────────────────
    bool   isWalkable(const finenav::Position3D& p) const override;
    double getStepSize()                            const override;

private:
    // ── 地图构建 ───────────────────────────────────────────────────────────
    void binarizeMap(const nav_msgs::msg::OccupancyGrid& msg, int threshold);
    void buildCostMap(double inflation_radius, double cost_scaling_factor);

    // ── 坐标转换 ───────────────────────────────────────────────────────────
    bool worldToGrid(double x, double y, int& col, int& row) const;

    // ── 代价计算辅助 ───────────────────────────────────────────────────────
    // 给定像素距离（单位：格），返回指数衰减代价 [0, 253]
    int computeExpCost(float dist_m) const;

    // ── 地图元数据 ─────────────────────────────────────────────────────────
    double   resolution_          = 0.05;
    uint32_t width_               = 0;
    uint32_t height_              = 0;
    double   origin_x_            = 0.0;
    double   origin_y_            = 0.0;
    double   cost_scaling_factor_ = 10.0;

    cv::Mat binary_map_; ///< 二值地图：254=障碍/未知，0=自由
    cv::Mat cost_map_;   ///< 指数衰减代价图 [0,253]，障碍格=254

    // ── OBB 碰撞模型 ───────────────────────────────────────────────────────
    // 尺寸硬编码 (L=0.8, W=0.7, H=0.3)，采样步长 0.1m，与 MPPI 一致
    mutable finenav::OBBCollisionModel obb_model_;

    // 中心点代价 >= 此阈值时才触发 OBB 精确碰撞检查
    // = 253 * exp(-cost_scaling_factor * obb_outer_radius)
    int possibly_inscribed_cost_ = 0;
};

} // namespace bfs
