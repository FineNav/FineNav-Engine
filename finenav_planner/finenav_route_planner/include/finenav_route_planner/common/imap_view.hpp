// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include "finenav_core/math_def.hpp"   // Position3D

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;

/**
 * @brief 面向用户的世界坐标地图视图接口
 * @details AstarPathSearch 与 BfsPathSearch 共用的统一接口。
 *          BFS 规划器可以忽略 getCost()（返回 0.0 即可）。
 * @details 实现该接口即可将你的地图接入栅格规划器，
 *          无需了解 Vec3、栅格单元或搜索算法内部的任何细节。
 * @details 你只需要提供两类信息：
 *          1. 世界坐标查询（isWalkable / getCost）：
 *             所有输入均为世界坐标（单位为米，WORLD_FIXED_FRAME 坐标系）。
 *          2. 搜索步长（getStepSize）：
 *             即"规划器每移动一个栅格，实际前进多少米？"
 *             - OccupancyGrid → 返回 info.resolution
 *             - 体素地图      → 返回 voxel_size
 *             - 自定义 SDF    → 返回期望的采样间隔
 * @note FineNav-Engine 中所有地图均使用 WORLD_FIXED_FRAME 坐标系，
 *       栅格原点恒为 (0, 0, 0)，无需另行指定
 */
class IMapView {
public:
    virtual ~IMapView() = default;

    // ── 世界坐标查询 ──────────────────────────────────────────
    /// 机器人能否通过世界坐标点 p？
    virtual bool isWalkable(const finenav::core::Position3D& p) const = 0;

    /// 点 p 处的额外通行代价（必须 >= 0）。
    /// A* 边权重 = step_cost * (1 + getCost(p))。
    /// 均匀代价地图返回 0.0 即可（例如与 BFS 搭配使用时）。
    virtual double getCost(const finenav::core::Position3D& p) const = 0;

    // ── 离散化提示 ────────────────────────────────────────────
    /// 规划器每走一个栅格步实际前进的距离（米）。
    virtual double getStepSize() const = 0;
};

} // namespace astar

