// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "finenav_engine/finenav_engine.hpp"
#include "grid_map.hpp" // TODO: resolve anti-pattern export

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "finenav_mppi_controller/controller.hpp"
#include "stvl_manager.hpp"
#include "terrain_mapview.hpp"
#include "finenav_util/cloud_publish_helper.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trim_by_distance.hpp"
#include "seek_to_nearest_point.hpp"
#include "trim_by_aabb.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace finenav;

namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

void assignPlanarOrientations(std::vector<geometry_msgs::msg::Pose>& poses)
{
    if (poses.size() < 2) {
        return;
    }

    for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
        const auto& from = poses[i].position;
        const auto& to = poses[i + 1].position;
        poses[i].orientation = yawToQuaternion(std::atan2(to.y - from.y, to.x - from.x));
    }
    poses.back().orientation = poses[poses.size() - 2].orientation;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void trimPathPrefixBehindRobot(
    finenav_msgs::msg::Trajectory& traj,
    const geometry_msgs::msg::Pose& robot_pose)
{
    using T = finenav_msgs::msg::Trajectory;
    if (!(traj.valid_fields & T::TRAJ_POSE) || traj.poses.size() < 2) {
        return;
    }

    const double yaw = yawFromQuaternion(robot_pose.orientation);
    const double forward_x = std::cos(yaw);
    const double forward_y = std::sin(yaw);

    constexpr double kForwardTolerance = -0.02;
    std::optional<std::size_t> first_forward_idx;
    for (std::size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& p = traj.poses[i].position;
        const double dx = p.x - robot_pose.position.x;
        const double dy = p.y - robot_pose.position.y;
        const double forward_projection = dx * forward_x + dy * forward_y;
        if (forward_projection >= kForwardTolerance) {
            first_forward_idx = i;
            break;
        }
    }

    if (!first_forward_idx.has_value() || *first_forward_idx == 0u) {
        return;
    }

    traj.poses.erase(traj.poses.begin(), traj.poses.begin() + static_cast<std::ptrdiff_t>(*first_forward_idx));
    assignPlanarOrientations(traj.poses);
}

class RouteTrackingProgress
{
public:
    void trimPassedPrefix(
        finenav_msgs::msg::Trajectory& traj,
        const geometry_msgs::msg::Pose& robot_pose)
    {
        using T = finenav_msgs::msg::Trajectory;
        if (!(traj.valid_fields & T::TRAJ_POSE) || traj.poses.size() < 2) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto stamp = rclcpp::Time(traj.header.stamp);
        if (!initialized_ || stamp != episode_stamp_) {
            initialized_ = true;
            episode_stamp_ = stamp;
            progress_idx_ = 0;
        }

        progress_idx_ = std::min(progress_idx_, traj.poses.size() - 1);

        std::size_t nearest_idx = progress_idx_;
        double nearest_dist_sq = std::numeric_limits<double>::max();
        for (std::size_t i = progress_idx_; i < traj.poses.size(); ++i) {
            const auto& p = traj.poses[i].position;
            const double dx = p.x - robot_pose.position.x;
            const double dy = p.y - robot_pose.position.y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < nearest_dist_sq) {
                nearest_dist_sq = dist_sq;
                nearest_idx = i;
            }
        }

        progress_idx_ = nearest_idx;
        if (progress_idx_ > 0u) {
            traj.poses.erase(
                traj.poses.begin(),
                traj.poses.begin() + static_cast<std::ptrdiff_t>(progress_idx_));
            assignPlanarOrientations(traj.poses);
        }
    }

private:
    std::mutex mutex_;
    bool initialized_{false};
    rclcpp::Time episode_stamp_{0, 0, RCL_ROS_TIME};
    std::size_t progress_idx_{0};
};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true); // Use time in /clock
    auto node = rclcpp::Node::make_shared("route_finenav", options); // TODO: inhirent options?

    auto finenav_engine = std::make_shared<FineNavEngine>(node);

    auto localizer = finenav_engine->getLocalizer();

    auto stvl_manager = std::make_shared<StvlManager>(node);

    // ------------------------------ Launch Localization ----------------------------------

    auto gt_pose_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        [localizer](const nav_msgs::msg::Odometry& msg) {
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose.pose = msg.pose.pose;

            static bool localizer_initialized = false;      // TODO: automatically iniitailize when receive the first pose_msg
            if (!localizer_initialized) {
                localizer->setInitialPose(pose_msg);
                localizer_initialized = true;
            } else {
                localizer->feedObservation(pose_msg);
            }
        }
    );


    // --------------------------------- Launch Mapping --------------------------------------
    auto map_server = finenav_engine->createMapResource<GridMap<Voxel>>(
        "my_map_server", 10.0, true, 100.0);


    // ============================================================
    // 可视化点云 Helpers（由 main 持有，生命周期覆盖所有使用方）
    // ============================================================

    // 1. /map_cloud : 地图中有效 Voxel 的高度点云（白色）
    auto map_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);
    auto map_cloud_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    map_cloud_helper->configure(map_cloud_pub, true, "map");

    // 2. passability_map : 地形可通行性（绿=可通行, 红=障碍）
    auto passability_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("passability_map", 10);
    auto passability_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    passability_helper->configure(passability_pub, true, "map");

    // 3. costmap_cloud : 膨胀后代价图（绿=低代价, 红=高代价）
    auto costmap_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("costmap_cloud", 10);
    auto costmap_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    costmap_helper->configure(costmap_pub, true, "map");

    // 4. local_map : SimpleGridMapView 局部地图（按代价着色）
    auto local_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 10);
    auto local_map_helper = std::make_shared<finenav_utils::CloudPublishHelper>();
    local_map_helper->configure(local_map_pub, false, "map");

    // ============================================================

    auto terrain_analyzer = std::make_shared<TerrainAnalyzer>(
        map_server, node, passability_helper, costmap_helper);

    map_server->registerPreUpdateHook([stvl_manager, localizer](GridMap<Voxel> & map) {
        const auto & state = localizer->getState();
        Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
        T_map_body.linear() = state.R.matrix();
        T_map_body.translation() = state.p;
        stvl_manager->PruneExpiredCells(map, T_map_body);
    });

    map_server->registerPostUpdateHook([node, map_cloud_helper, terrain_analyzer, localizer](GridMap<Voxel> & map) {

        terrain_analyzer->update(localizer->getState().p.z(), map);

        // /map_cloud 可视化：所有有效 Voxel，高度映射到颜色（蓝低红高，简化为白色）
        if (map_cloud_helper->hasSubscribers()) {
            std::vector<Index> valid_indices;
            map.selectCellsByCondition(valid_indices, [](const Voxel & v) {
                return std::isfinite(v.z_height);
            });
            if (!valid_indices.empty()) {
                map_cloud_helper->reserve(valid_indices.size());
                for (const auto & idx : valid_indices) {
                    const auto pos = map.getPosition(idx);
                    const auto & cell = map.at(idx);
                    map_cloud_helper->addPoint(
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        cell.z_height,
                        {255, 255, 255}); // 白色
                }
                map_cloud_helper->publish(node->now());
            }
        }
    });

    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;

    auto laser_injector = map_server->addUpdateSource<pcl::PointCloud<pcl::PointXYZ>>(
        "front_laser",
        policy,
        [stvl_manager, localizer](
            const pcl::PointCloud<pcl::PointXYZ> & cloud,
            const rclcpp::Time & stamp,       // 点云的原始采集时间戳
            GridMap<Voxel> & map)
        {
            // 使用点云采集时刻的历史位姿，消除点云与位姿时间不同步导致的旋转漂移。
            // 若该时刻已超出历史窗口或 localizer 尚未初始化，则降级到最新位姿。
            const NavStateD state = [&]() -> NavStateD {
                if (auto s = localizer->getStateAt(stamp)) {
                    return *s;
                }
                return localizer->getState();
            }();

            Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
            T_map_body.linear()      = state.R.matrix();
            T_map_body.translation() = state.p;
            stvl_manager->InsertPointCloud(cloud, map, T_map_body);
        });

    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_for_lio",
        10,
        [node, laser_injector](const sensor_msgs::msg::PointCloud2 & msg) {
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(msg, pcl_cloud);
            laser_injector(std::move(pcl_cloud), rclcpp::Time(msg.header.stamp));
        });



    // --------------------------------- Launch Planning --------------------------------------
    using MppiPlanners = PlannerSet<nav2_mppi_controller::MPPIController>;
    auto my_control_layer = finenav_engine->createControlLayer<MppiPlanners>(
        "mppi_layer", finenav::TrackingPolicy{20.0});

    // /cmd_vel publisher — 由 post_plan 回调驱动，在每次规划成功后取第一步速度指令
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // [DEBUG] 发布裁减后的参考轨迹，供 RViz 可视化验证
    auto trimmed_path_pub = node->create_publisher<nav_msgs::msg::Path>("/trimmed_ref_path", 10);
    auto route_progress = std::make_shared<RouteTrackingProgress>();

    Region3D map_bounds;
    {
        map_bounds = map_server->getLockedReadView()->getWindowBounds();
    }

    my_control_layer->set_on_pre_plan(
    [map_bounds, map_server, localizer, node, trimmed_path_pub, route_progress](finenav::PrePlanContext& ctx) -> bool {
        // Route mode receives a complete episode path from compute_path_bridge:
        // current pose connector + route graph path. Keep it fixed during the
        // episode so topology and first-mile connection are not recomputed at 20 Hz.
        const auto& rp = ctx.robot_state.pose.position;
        route_progress->trimPassedPrefix(ctx.ref_traj, ctx.robot_state.pose);
        trimPathPrefixBehindRobot(ctx.ref_traj, ctx.robot_state.pose);

        // 1. Trim trailing waypoints beyond the maximum tracking distance
        //    (use the shorter dimension of the map as a conservative bound)
        // const auto bounds_size = map_bounds.max() - map_bounds.min();
        // const double max_dist = std::min(bounds_size.x(), bounds_size.y());
        // ctx.ref_traj | finenav::TrimByDistance(max_dist);

        // 2. Trim waypoints that fall outside the current map AABB
        const auto current_bounds = map_server->getLockedReadView()->getWindowBounds();
        ctx.ref_traj | finenav::TrimByAABB(current_bounds);

        if (ctx.ref_traj.poses.empty()) {
            RCLCPP_ERROR(
                node->get_logger(),
                "Reference trajectory is empty after trimming; robot=(%.3f, %.3f, %.3f), "
                "bounds x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f].",
                rp.x, rp.y, rp.z,
                current_bounds.min().x(), current_bounds.max().x(),
                current_bounds.min().y(), current_bounds.max().y(),
                current_bounds.min().z(), current_bounds.max().z());
            return false;
        }

        // [DEBUG] 发布裁减后的参考轨迹供 RViz 可视化验证
        {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = node->now();
            path_msg.header.frame_id = "map";
            for (const auto& pose : ctx.ref_traj.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path_msg.header;
                ps.pose = pose;
                path_msg.poses.push_back(ps);
            }
            trimmed_path_pub->publish(path_msg);
        }

        return true;
    });

    my_control_layer->set_on_post_plan(
        [cmd_vel_pub](finenav::PostPlanContext& ctx) -> bool {
            if (ctx.is_success &&
                !ctx.result_traj.twists.empty())
            {
                cmd_vel_pub->publish(ctx.result_traj.twists[0]);
            }
            return ctx.is_success;
        });

    // local_map_helper 捕获到 builder lambda，每次构造 SimpleGridMapView 时传入
    // robot_state_provider 和 should_stop_predicate 已由 FineNavEngine::createControlLayer() 自动注入。
    my_control_layer->bind_map_view(
        map_server,
        [node, terrain_analyzer, local_map_helper](const GridMap<Voxel>& map) {
            return std::make_shared<SimpleGridMapView>(map, node, terrain_analyzer, local_map_helper);
        });


    finenav_engine->registerPipeline(
        FineNavEngine::PipelineIO{
            .input_topic = "line_path",
            .output_topic = "mppi_layer_plan",
        },
        my_control_layer);


    // --------------------------------- Main Loop --------------------------------------------

    rclcpp::Rate loop_rate(200); // 200Hz
    auto nav_state_pub = node->create_publisher<nav_msgs::msg::Odometry>("nav_state", 10);

    while (rclcpp::ok()) {
        auto robot_state = localizer->getState();

        // Initialize with IIFE
        const auto nav_state_msg = [&node, &robot_state] {
            nav_msgs::msg::Odometry nav_state_msg;
            nav_state_msg.header.stamp = node->now();
            nav_state_msg.header.frame_id = "map";
            nav_state_msg.child_frame_id = "base_link";

            nav_state_msg.pose.pose.position.x = robot_state.p.x();
            nav_state_msg.pose.pose.position.y = robot_state.p.y();
            nav_state_msg.pose.pose.position.z = robot_state.p.z();

            auto quat = robot_state.R.unit_quaternion();
            nav_state_msg.pose.pose.orientation.x = quat.x();
            nav_state_msg.pose.pose.orientation.y = quat.y();
            nav_state_msg.pose.pose.orientation.z = quat.z();
            nav_state_msg.pose.pose.orientation.w = quat.w();

            nav_state_msg.twist.twist.linear.x = robot_state.v.x();
            nav_state_msg.twist.twist.linear.y = robot_state.v.y();
            nav_state_msg.twist.twist.linear.z = robot_state.v.z();
            nav_state_msg.twist.twist.angular.x = robot_state.omega.x();
            nav_state_msg.twist.twist.angular.y = robot_state.omega.y();
            nav_state_msg.twist.twist.angular.z = robot_state.omega.z();

            return nav_state_msg;
        }();
        nav_state_pub->publish(nav_state_msg);


        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
