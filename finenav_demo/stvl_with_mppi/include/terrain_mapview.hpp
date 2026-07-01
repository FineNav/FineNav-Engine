// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"
#include "terrain_analysis.hpp"
#include "finenav_util/cloud_publish_helper.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
using namespace mppi;
using std::chrono_literals::operator"" ms;

/// 不可变 OBB 配置，构造后只读 → 多线程安全
struct ObbConfig {
    finenav::Vector3D min{-0.4, -0.35, -0.15};
    finenav::Vector3D max{ 0.4,  0.35,  0.15};
    finenav::Pose     local_pose = finenav::Pose::Identity();
    finenav::OBBCollisionPolicy policy{0.1};
    double outer_radius = 0.0;                    // 预计算外径/2
};

class SimpleGridMapView : public nav2_mppi_controller::IMapView {
public:
    /**
     * @param local_map_helper 可选，由外部（main）持有并配置好，用于发布局部地图点云。
     *                         传 nullptr 则不发布。
     */
    explicit SimpleGridMapView(
        const finenav::GridMap<Voxel>& map,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<TerrainAnalyzer> terrain_analyzer,
        ObbConfig obb_config,
        std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper = nullptr);

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;
    bool isCollision(float x, float y, float theta) const override;
    int getCost(const finenav::Position3D& pos) const override;
    float getRadius() const override;
    float costAtPose(float x, float y, float theta) const override;
    std::string getBaseFrameID() const override;

    void publishLocalMap();

    /// 每线程独立 OBB，首次调用懒创建。零竞争。
    static finenav::OBBCollisionModel& getThreadObb(const ObbConfig& cfg) {
        thread_local std::unique_ptr<finenav::OBBCollisionModel> obb;
        if (!obb) {
            obb = std::make_unique<finenav::OBBCollisionModel>(
                cfg.min, cfg.max, cfg.local_pose, cfg.policy);
        }
        return *obb;
    }

private:
    int ComputeCost(int distance) const;
    finenav::Pose createPose2D(double x, double y, double theta) const;

    std::shared_ptr<TerrainAnalyzer> terrain_analyzer_;
    const finenav::GridMap<Voxel>& map_;
    double inscribed_radius_;
    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
    Eigen::ArrayXXf ground_array_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper_;

    ObbConfig obb_config_;  // 构造时填充，之后只读
};
