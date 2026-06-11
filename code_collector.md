# 代码库文档

生成时间: 2026-06-07 10:48:37

文件总数: 190

---

## 目录结构

- `.pre-commit-config.yaml`
- `finenav_collision_model/CMakeLists.txt`
- `finenav_collision_model/include/finenav_collision_model/collision_model.hpp`
- `finenav_collision_model/include/finenav_collision_model/obb_collision_model.hpp`
- `finenav_collision_model/package.xml`
- `finenav_collision_model/src/obb_collision_model.cpp`
- `finenav_collision_model/test/obb_collision_model_test.cpp`
- `finenav_core/CMakeLists.txt`
- `finenav_core/include/finenav_core/concepts.hpp`
- `finenav_core/include/finenav_core/map/locked_map_view.hpp`
- `finenav_core/include/finenav_core/map/map_server.hpp`
- `finenav_core/include/finenav_core/map/map_server_impl.hpp`
- `finenav_core/include/finenav_core/map/update_source.hpp`
- `finenav_core/include/finenav_core/plan/control_layer.hpp`
- `finenav_core/include/finenav_core/plan/control_layer_impl.hpp`
- `finenav_core/include/finenav_core/plan/data_packet.hpp`
- `finenav_core/include/finenav_core/plan/execution_policy.hpp`
- `finenav_core/include/finenav_core/plan/plan_context.hpp`
- `finenav_core/include/finenav_core/plan/planner_set.hpp`
- `finenav_core/include/finenav_core/plan/ros_conversions.hpp`
- `finenav_core/package.xml`
- `finenav_demo/CMakeLists.txt`
- `finenav_demo/package.xml`
- `finenav_demo/stvl_with_mppi/CMakeLists.txt`
- `finenav_demo/stvl_with_mppi/config/finenav_params.yaml`
- `finenav_demo/stvl_with_mppi/config/mppi_params.yaml`
- `finenav_demo/stvl_with_mppi/include/stvl_manager.hpp`
- `finenav_demo/stvl_with_mppi/include/terrain_analysis.hpp`
- `finenav_demo/stvl_with_mppi/include/terrain_mapview.hpp`
- `finenav_demo/stvl_with_mppi/include/voxel.hpp`
- `finenav_demo/stvl_with_mppi/launch/navigation_stack.launch.py`
- `finenav_demo/stvl_with_mppi/launch/stvl_with_mppi.launch.py`
- `finenav_demo/stvl_with_mppi/src/adapter.py`
- `finenav_demo/stvl_with_mppi/src/goal_pose_bridge_node.cpp`
- `finenav_demo/stvl_with_mppi/src/hello_finenav.cpp`
- `finenav_demo/stvl_with_mppi/src/map.yaml`
- `finenav_demo/stvl_with_mppi/src/nav_manager.py`
- `finenav_demo/stvl_with_mppi/src/stvl_manager.cpp`
- `finenav_demo/stvl_with_mppi/src/terrain_analysis.cpp`
- `finenav_demo/stvl_with_mppi/src/terrain_mapview.cpp`
- `finenav_engine/CMakeLists.txt`
- `finenav_engine/include/finenav_engine/finenav_engine.hpp`
- `finenav_engine/package.xml`
- `finenav_interface/CMakeLists.txt`
- `finenav_interface/include/frustum3d.hpp`
- `finenav_interface/include/simpleTestMap.hpp`
- `finenav_interface/include/simpleTestPlanner.hpp`
- `finenav_interface/package.xml`
- `finenav_interface/src/grid_map_stvl.cpp`
- `finenav_interface/src/test.cpp`
- `finenav_interface/src/test_map.yaml`
- `finenav_interface/src/test_planner.yaml`
- `finenav_localizer/CMakeLists.txt`
- `finenav_localizer/eval/CMakeLists.txt`
- `finenav_localizer/eval/eval_config.hpp`
- `finenav_localizer/eval/eval_scenario.cpp`
- `finenav_localizer/eval/plot.py`
- `finenav_localizer/include/finenav_localizer/eskf.hpp`
- `finenav_localizer/include/finenav_localizer/filter_model.hpp`
- `finenav_localizer/include/finenav_localizer/finenav_localizer.hpp`
- `finenav_localizer/include/finenav_localizer/kalman_filter/kalman_filter.hpp`
- `finenav_localizer/include/finenav_localizer/kalman_filter/time_delay_kalman_filter.hpp`
- `finenav_localizer/include/finenav_localizer/nav_state_buffer.hpp`
- `finenav_localizer/include/finenav_localizer/observation_queue.hpp`
- `finenav_localizer/package.xml`
- `finenav_localizer/src/finenav_localizer.cpp`
- `finenav_localizer/src/finenav_localizer.yaml`
- `finenav_localizer/test/CMakeLists.txt`
- `finenav_localizer/test/test_localizer_units.cpp`
- `finenav_map/grid_map/CMakeLists.txt`
- `finenav_map/grid_map/include/grid_map.hpp`
- `finenav_map/grid_map/include/grid_map_impl.hpp`
- `finenav_map/grid_map/include/grid_map_iterator.hpp`
- `finenav_map/grid_map/include/grid_map_math.hpp`
- `finenav_map/grid_map/package.xml`
- `finenav_map/grid_map/param/grid_map.yaml`
- `finenav_motion_decorator/CMakeLists.txt`
- `finenav_motion_decorator/include/detail/trim_utils.hpp`
- `finenav_motion_decorator/include/i_motion_decorator.hpp`
- `finenav_motion_decorator/include/seek_to_nearest_point.hpp`
- `finenav_motion_decorator/include/trim_by_aabb.hpp`
- `finenav_motion_decorator/include/trim_by_count.hpp`
- `finenav_motion_decorator/include/trim_by_distance.hpp`
- `finenav_motion_decorator/include/trim_by_duration.hpp`
- `finenav_motion_decorator/package.xml`
- `finenav_motion_decorator/src/seek_to_nearest_point.cpp`
- `finenav_motion_decorator/src/trim_by_aabb.cpp`
- `finenav_motion_decorator/src/trim_by_count.cpp`
- `finenav_motion_decorator/src/trim_by_distance.cpp`
- `finenav_motion_decorator/src/trim_by_duration.cpp`
- `finenav_msgs/CMakeLists.txt`
- `finenav_msgs/package.xml`
- `finenav_navigator/CMakeLists.txt`
- `finenav_navigator/behavior_trees/navigate_to_pose.xml`
- `finenav_navigator/behavior_trees/test_tree.xml`
- `finenav_navigator/include/finenav_navigator/behavior_tree_engine.hpp`
- `finenav_navigator/include/finenav_navigator/navigator.hpp`
- `finenav_navigator/include/finenav_navigator/plugins/action/compute_plan.hpp`
- `finenav_navigator/include/finenav_navigator/plugins/action/publish_zero_cmd_vel.hpp`
- `finenav_navigator/include/finenav_navigator/plugins/condition/is_goal_reached.hpp`
- `finenav_navigator/include/finenav_navigator/plugins/condition/is_stuck.hpp`
- `finenav_navigator/package.xml`
- `finenav_navigator/src/behavior_tree_engine.cpp`
- `finenav_navigator/src/finenav_navigator.yaml`
- `finenav_navigator/src/naviagtor.cpp`
- `finenav_navigator/src/plugins/action/compute_plan.cpp`
- `finenav_navigator/src/plugins/action/publish_zero_cmd_vel.cpp`
- `finenav_navigator/src/plugins/condition/conditions.cpp`
- `finenav_planner/finenav_mppi_controller/CMakeLists.txt`
- `finenav_planner/finenav_mppi_controller/LICENSE.md`
- `finenav_planner/finenav_mppi_controller/benchmark/CMakeLists.txt`
- `finenav_planner/finenav_mppi_controller/benchmark/controller_benchmark.cpp`
- `finenav_planner/finenav_mppi_controller/benchmark/optimizer_benchmark.cpp`
- `finenav_planner/finenav_mppi_controller/critics.xml`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/controller.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critic_data.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critic_function.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critic_manager.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/constraint_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/cost_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/goal_angle_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/goal_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/obstacles_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_align_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_align_legacy_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_angle_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_follow_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/prefer_forward_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/twirling_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/velocity_deadband_critic.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/map_view_def.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/constraints.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/control_sequence.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/optimizer_settings.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/path.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/state.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/trajectories.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/motion_models.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/optimizer.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/tools/noise_generator.hpp`
- `finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/tools/utils.hpp`
- `finenav_planner/finenav_mppi_controller/mppic.xml`
- `finenav_planner/finenav_mppi_controller/package.xml`
- `finenav_planner/finenav_mppi_controller/src/controller.cpp`
- `finenav_planner/finenav_mppi_controller/src/critic_manager.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/constraint_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/cost_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/goal_angle_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/goal_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/obstacles_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/path_align_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/path_align_legacy_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/path_angle_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/path_follow_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/prefer_forward_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/twirling_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/critics/velocity_deadband_critic.cpp`
- `finenav_planner/finenav_mppi_controller/src/finenav_mppi_controller.yaml`
- `finenav_planner/finenav_mppi_controller/src/noise_generator.cpp`
- `finenav_simulator/CMakeLists.txt`
- `finenav_simulator/config/fastlio_gazebo_config.yaml`
- `finenav_simulator/config/ros_gz_bridge.yaml`
- `finenav_simulator/config/world_spawn_poses.yaml`
- `finenav_simulator/include/finenav_simulator/ground_truth_map_localizer.hpp`
- `finenav_simulator/include/finenav_simulator/keyboard_teleop.hpp`
- `finenav_simulator/include/finenav_simulator/odom_path_publisher.hpp`
- `finenav_simulator/include/finenav_simulator/point_cloud_converter.hpp`
- `finenav_simulator/include/finenav_simulator/simulator_helper_node.hpp`
- `finenav_simulator/launch/complete/finenav_simulator.launch.py`
- `finenav_simulator/launch/complete/simulator_infrastructure.launch.py`
- `finenav_simulator/launch/complete/slam_and_localization.launch.py`
- `finenav_simulator/launch/complete/spawn_world_and_robot.launch.py`
- `finenav_simulator/launch/core/gazebo.launch.py`
- `finenav_simulator/launch/core/pointcloud_complete_launch.py`
- `finenav_simulator/launch/core/robot.launch.py`
- `finenav_simulator/package.xml`
- `finenav_simulator/scripts/sdf_launch_utils.py`
- `finenav_simulator/src/ground_truth_map_localizer.cpp`
- `finenav_simulator/src/keyboard_teleop.cpp`
- `finenav_simulator/src/odom_path_publisher.cpp`
- `finenav_simulator/src/point_cloud_converter.cpp`
- `finenav_simulator/src/simulator_helper_node.cpp`
- `finenav_util/CMakeLists.txt`
- `finenav_util/include/finenav_util/algo_configurator.hpp`
- `finenav_util/include/finenav_util/cloud_publish_helper.hpp`
- `finenav_util/include/finenav_util/node_thread.hpp`
- `finenav_util/include/finenav_util/simple_action_server.hpp`
- `finenav_util/package.xml`
- `finenav_util/src/cloud_publish_helper.cpp`
- `finenav_util/src/node_thread.cpp`

---

## 文件内容

### 📄 .pre-commit-config.yaml

```yaml

exclude: ".pgm$|.svg$|.pcd$"

repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v6.0.0
    hooks:
      - id: check-added-large-files
      - id: check-ast
      - id: check-case-conflict
      - id: check-merge-conflict
      - id: check-symlinks
      - id: check-xml
      - id: check-yaml
      - id: debug-statements
      - id: end-of-file-fixer
      - id: forbid-submodules
      - id: mixed-line-ending
      - id: trailing-whitespace
      - id: fix-byte-order-marker

  - repo: https://github.com/pycqa/isort
    rev: 8.0.1
    hooks:
      - id: isort
        args: ["--profile", "black", "--filter-files"]

  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: v22.1.0
    hooks:
      - id: clang-format
        args: ["-i", "-style=file"]
```

---

### 📄 finenav_collision_model/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_collision_model)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} SHARED
    src/obb_collision_model.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
    ament_add_gtest(${PROJECT_NAME}_test
        test/obb_collision_model_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif ()

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    eigen3_cmake_module
    Eigen3
)

ament_auto_package()
```

---

### 📄 finenav_collision_model/include/finenav_collision_model/collision_model.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Dense>
#include <concepts>
#include <type_traits>

namespace finenav {

using Vector3D    = Eigen::Vector3d;
using Position3D  = Eigen::Vector3d;
using Orientation = Eigen::Quaterniond;
using Region3D    = Eigen::AlignedBox3d;
using Pose        = Eigen::Isometry3d;

using Vector   = Eigen::Vector3d;
using Position = Eigen::Vector3d;
using Index    = Eigen::Vector3i;
using Size     = Eigen::Vector3i;
using Length   = Eigen::Vector3d;

/**
 * @brief 碰撞规则 Concept
 * @details 任何可被调用、接受 const Position3D& 并返回 bool 的可调用对象均满足此约束。
 */
template <typename F>
concept CollisionRule =
    std::invocable<F, const Position3D&> &&
    std::same_as<std::invoke_result_t<F, const Position3D&>, bool>;

/**
 * @brief 代价规则 Concept
 * @details 任何可被调用、接受 const Position3D& 并返回 int 的可调用对象均满足此约束。
 */
template <typename F>
concept CostRule =
    std::invocable<F, const Position3D&> &&
    std::same_as<std::invoke_result_t<F, const Position3D&>, int>;

} // namespace finenav
```

---

### 📄 finenav_collision_model/include/finenav_collision_model/obb_collision_model.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_collision_model/collision_model.hpp"
#include <stdexcept>
#include <vector>

namespace finenav {

/**
 * @brief OBBCollisionModel 的参数
 * @param step 表面采样步长（单位：m），控制碰撞检测的空间精度，由用户决定，default=0.1m
 */
struct OBBCollisionPolicy {
    double step = 0.1; ///< 表面采样步长（m）
};

/**
 * @brief OBB (Oriented Bounding Box) 碰撞模型
 *
 * @details
 * 碰撞判断完全由调用方通过 CollisionRule 提供，因此可适配任意地图类型
 * 碰撞检测分为两阶段以支持各种场景:
 *   - precompute(pose)：          在机器人位姿改变时调用一次，缓存世界系下的所有采样点。
 *   - checkCollision(rule)：      高频接口，直接遍历缓存点，零几何变换开销，需先调用 precompute。
 *   - checkCollision(pose, rule)：低频接口，内部自动 precompute，适用于普通调用。
 */
class OBBCollisionModel {
public:
    using Policy = OBBCollisionPolicy;

    /**
     * @brief 构造函数
     * @param size      OBB 在三个轴方向的尺寸 (L, W, H)
     * @param local_pose OBB 在机器人本体坐标系下的位姿（含旋转和偏移），默认为单位变换
     * @param policy    采样策略参数
     */
    explicit OBBCollisionModel(const Vector3D& size,
                               const Pose& local_pose = Pose::Identity(),
                               const Policy& policy   = Policy{});

    /**
     * @brief 预计算并缓存世界坐标系下的所有表面采样点
     * @details 只需在机器人位姿改变时调用一次。
     *          调用后可多次调用 checkCollision(rule) 而无需重复几何变换。
     * @param pose 机器人在世界坐标系下的位姿
     */
    void precompute(const Pose& pose);

    /**
     * @brief 高频碰撞检测（使用缓存采样点）
     * @details 调用前须先调用 precompute(pose)，适用于 MPPI 等需要极低延迟的场景。
     * @note 此接口为 const。低频重载因内部调用 precompute（修改缓存）而为非 const，
     *       须在对象变为 const 之前由外部提前调用 precompute(pose) 填充缓存。
     * @tparam CR 满足 CollisionRule concept 的可调用对象
     * @param rule 碰撞判断函数，由调用方提供
     * @return 任意一个采样点触发 rule 返回 true 时，整体返回 true
     */
    template <CollisionRule CR>
    bool checkCollision(CR&& rule) const;

    /**
     * @brief 低频碰撞检测（自动 precompute）
     * @details 内部自动调用 precompute(pose) 后执行检测，无需手动管理缓存，适用于普通场景。
     * @tparam CR 满足 CollisionRule concept 的可调用对象
     * @param pose 机器人在世界坐标系下的位姿
     * @param rule 碰撞判断函数
     * @return 如果发生碰撞返回 true
     */
    template <CollisionRule CR>
    bool checkCollision(const Pose& pose, CR&& rule);

    /**
     * @brief 高频代价检测（使用缓存采样点）
     * @details 调用前须先调用 precompute(pose)，遍历所有表面采样点并返回最高代价值。
     * @tparam CR 满足 CostRule concept 的可调用对象
     * @param rule 代价计算函数，由调用方提供
     * @return 所有采样点中的最高代价值
     */
    template <CostRule CR>
    int checkCost(CR&& rule) const;

    /**
     * @brief 低频代价检测（自动 precompute）
     * @details 内部自动调用 precompute(pose) 后执行代价计算，无需手动管理缓存。
     * @tparam CR 满足 CostRule concept 的可调用对象
     * @param pose 机器人在世界坐标系下的位姿
     * @param rule 代价计算函数
     * @return 所有采样点中的最高代价值
     */
    template <CostRule CR>
    int checkCost(const Pose& pose, CR&& rule);

    /**
     * @brief 获取 OBB 外径
     * @details 外径定义为包围该 OBB 的最小球直径，数值等于 OBB 空间对角线长度。
     * @return OBB 外径
     */
    double getOuterDiameter() const;

private:
    Vector3D size_;           ///< OBB 尺寸
    double   outer_diameter_; ///< OBB 外径（空间对角线长度）
    Vector3D min_;            ///< OBB 自身坐标系下的最小边界
    Vector3D max_;            ///< OBB 自身坐标系下的最大边界
    Pose     local_pose_;     ///< OBB 在机器人本体坐标系下的位姿（旋转 + 偏移）
    Policy   policy_;         ///< 采样策略参数

    std::vector<Position3D> cached_points_; ///< 缓存的世界系采样点（由 precompute 填充）
};

template <CollisionRule CR>
bool OBBCollisionModel::checkCollision(CR&& rule) const {
    if (cached_points_.empty()) {
        throw std::logic_error(
            "OBBCollisionModel::checkCollision called before precompute");
    }
    for (const auto& pt : cached_points_) {
        if (rule(pt)) {
            return true;
        }
    }
    return false;
}

template <CollisionRule CR>
bool OBBCollisionModel::checkCollision(const Pose& pose, CR&& rule) {
    precompute(pose);
    return checkCollision(std::forward<CR>(rule));
}

template <CostRule CR>
int OBBCollisionModel::checkCost(CR&& rule) const {
    if (cached_points_.empty()) {
        throw std::logic_error(
            "OBBCollisionModel::checkCost called before precompute");
    }
    int max_cost = 0;
    for (const auto& pt : cached_points_) {
        int cost = rule(pt);
        if (cost > max_cost) {
            max_cost = cost;
        }
    }
    return max_cost;
}

template <CostRule CR>
int OBBCollisionModel::checkCost(const Pose& pose, CR&& rule) {
    precompute(pose);
    return checkCost(std::forward<CR>(rule));
}

} // namespace finenav
```

---

### 📄 finenav_collision_model/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_collision_model</name>
    <version>0.1.0</version>
    <description>Collision model library for FineNav2D</description>
    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake_auto</buildtool_depend>

    <depend>eigen3_cmake_module</depend>

    <test_depend>ament_cmake_gtest</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>

```

---

### 📄 finenav_collision_model/src/obb_collision_model.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_collision_model/obb_collision_model.hpp"
#include <stdexcept>

namespace finenav {

OBBCollisionModel::OBBCollisionModel(const Vector3D& size, const Pose& local_pose, const Policy& policy)
    : size_(size), outer_diameter_(0.0), local_pose_(local_pose), policy_(policy) {
    if (policy_.step <= 0.0) {
        throw std::invalid_argument(
            "OBBCollisionModel: policy.step must be greater than 0");
    }
    min_ = -size_ / 2.0;
    max_ =  size_ / 2.0;
    outer_diameter_ = (max_ - min_).norm();
}

double OBBCollisionModel::getOuterDiameter() const {
    return outer_diameter_;
}

void OBBCollisionModel::precompute(const Pose& pose) {
    cached_points_.clear();

    const double step = policy_.step;

    // 预先合并两次变换：OBB 本体系 → 机器人本体系 → 世界系
    // 避免在每个采样点重复计算 pose * local_pose_
    const Pose body_to_world = pose * local_pose_;

    // 遍历 OBB 的 6 个面（每个轴各 min/max 两个面）进行表面采样
    // 轴旋转技巧：对于固定轴 axis，u = (axis+1)%3，v = (axis+2)%3
    for (int axis = 0; axis < 3; ++axis) {
        const int u_axis = (axis + 1) % 3;
        const int v_axis = (axis + 2) % 3;

        for (const double side : {min_[axis], max_[axis]}) {
            for (double u = min_[u_axis]; u <= max_[u_axis]; u += step) {
                for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                    Position3D local_pt;
                    local_pt[axis]   = side;
                    local_pt[u_axis] = u;
                    local_pt[v_axis] = v;
                    cached_points_.push_back(body_to_world * local_pt);
                }
                // 补全 v 方向最大边界（浮点步进可能漏掉 max_v）
                Position3D edge_v;
                edge_v[axis]   = side;
                edge_v[u_axis] = u;
                edge_v[v_axis] = max_[v_axis];
                cached_points_.push_back(body_to_world * edge_v);
            }
            // 补全 u 方向最大边界列（浮点步进可能漏掉 max_u）
            for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                Position3D edge_u;
                edge_u[axis]   = side;
                edge_u[u_axis] = max_[u_axis];
                edge_u[v_axis] = v;
                cached_points_.push_back(body_to_world * edge_u);
            }
            // 补全 (max_u, max_v) 角点（edge_v 和 edge_u 均未覆盖此交叉点）
            Position3D corner;
            corner[axis]   = side;
            corner[u_axis] = max_[u_axis];
            corner[v_axis] = max_[v_axis];
            cached_points_.push_back(body_to_world * corner);
        }
    }
}

} // namespace finenav
```

---

### 📄 finenav_collision_model/test/obb_collision_model_test.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include "finenav_collision_model/obb_collision_model.hpp"
#include <cmath>

using namespace finenav;

// ─── 辅助：始终返回 false 的 rule（无障碍） ───────────────────────────
auto free_rule  = [](const Position3D&) { return false; };
// 始终返回 true 的 rule（全障碍）
auto block_rule = [](const Position3D&) { return true;  };

// ─── 1. 非法参数：step <= 0 抛出异常 ─────────────────────────────────
TEST(OBBCollisionModelTest, InvalidStepThrows) {
    OBBCollisionPolicy bad_policy{0.0};
    EXPECT_THROW(OBBCollisionModel({1.0, 1.0, 1.0}, Pose::Identity(), bad_policy),
                 std::invalid_argument);

    OBBCollisionPolicy neg_policy{-0.1};
    EXPECT_THROW(OBBCollisionModel({1.0, 1.0, 1.0}, Pose::Identity(), neg_policy),
                 std::invalid_argument);
}

// ─── 2. 外径等于 OBB 空间对角线长度 ─────────────────────────────────
TEST(OBBCollisionModelTest, GetOuterDiameterReturnsBoxDiagonalLength) {
    OBBCollisionModel model({3.0, 4.0, 12.0});
    EXPECT_DOUBLE_EQ(model.getOuterDiameter(), 13.0);
}

// ─── 3. 未调用 precompute 直接 checkCollision 抛出异常 ──────────────
TEST(OBBCollisionModelTest, CheckCollisionBeforePrecomputeThrows) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    EXPECT_THROW(model.checkCollision(free_rule), std::logic_error);
}

// ─── 4. 无碰撞：rule 全返回 false ────────────────────────────────────
TEST(OBBCollisionModelTest, NoCollisionWhenFree) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    EXPECT_FALSE(model.checkCollision(pose, free_rule));
}

// ─── 5. 有碰撞：rule 全返回 true ─────────────────────────────────────
TEST(OBBCollisionModelTest, CollisionWhenBlocked) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    EXPECT_TRUE(model.checkCollision(pose, block_rule));
}

// ─── 6. 高频接口：precompute 后多次 checkCollision ───────────────────
TEST(OBBCollisionModelTest, HighFreqInterfaceReusesCache) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 多次调用应返回相同结果
    EXPECT_FALSE(model.checkCollision(free_rule));
    EXPECT_FALSE(model.checkCollision(free_rule));
    EXPECT_TRUE(model.checkCollision(block_rule));
}

// ─── 7. 障碍在 OBB 范围内才碰撞（位置相关 rule）────────────────────
TEST(OBBCollisionModelTest, CollisionOnlyWhenObstacleIntersectsOBB) {
    // OBB: 1x1x1 m，中心在原点，半轴长 0.5m，X 面表面点 x = ±0.5
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 障碍在 x=2.0，远离 OBB → 无碰撞
    auto far_rule = [](const Position3D& p) { return p.x() > 1.9; };
    EXPECT_FALSE(model.checkCollision(pose, far_rule));

    // 障碍在 x=0.5（OBB 表面），预计有采样点触碰
    auto surface_rule = [](const Position3D& p) { return std::abs(p.x() - 0.5) < 1e-6; };
    EXPECT_TRUE(model.checkCollision(pose, surface_rule));
}

// ─── 8. OBB 带偏移的 local_pose ──────────────────────────────────────
TEST(OBBCollisionModelTest, LocalPoseOffset) {
    // OBB 在机器人本体系中偏移 (5, 0, 0)
    Pose local_pose = Pose::Identity();
    local_pose.translation() = Vector3D(5.0, 0.0, 0.0);

    OBBCollisionModel model({1.0, 1.0, 1.0}, local_pose);
    Pose robot_pose = Pose::Identity(); // 机器人在原点

    // OBB 中心在世界系 (5, 0, 0)，半轴长 0.5m，X 面表面点 x = 4.5 / 5.5
    // 障碍在原点附近 → 无碰撞
    auto near_origin = [](const Position3D& p) { return p.norm() < 0.6; };
    EXPECT_FALSE(model.checkCollision(robot_pose, near_origin));

    // 障碍在 x=5.0 附近 → 有碰撞
    auto near_offset = [](const Position3D& p) { return std::abs(p.x() - 5.0) < 0.6; };
    EXPECT_TRUE(model.checkCollision(robot_pose, near_offset));
}

// ─── 9. OBB 带旋转的 local_pose（绕 Z 轴旋转 90°）────────────────────
TEST(OBBCollisionModelTest, LocalPoseRotation) {
    // OBB 2x1x1（长轴沿 X），绕 Z 旋转 90° 后长轴变为 Y 方向
    Pose local_pose = Pose::Identity();
    local_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Vector3D::UnitZ()).toRotationMatrix();

    OBBCollisionModel model({2.0, 1.0, 1.0}, local_pose);
    Pose robot_pose = Pose::Identity();

    // 旋转后 OBB 长轴在 Y，Y 方向表面点应在 ±1.0
    // x 方向表面点应在 ±0.5（原来的 Y 半轴 = 0.5）
    auto y_surface = [](const Position3D& p) { return std::abs(std::abs(p.y()) - 1.0) < 1e-6; };
    EXPECT_TRUE(model.checkCollision(robot_pose, y_surface));

    // X 方向不应有到 ±1.0 的采样点（原长轴已旋转到 Y）
    auto x_far = [](const Position3D& p) { return std::abs(p.x()) > 0.9; };
    EXPECT_FALSE(model.checkCollision(robot_pose, x_far));
}

// ─── 10. 机器人移动后采样点跟随更新 ─────────────────────────────────
TEST(OBBCollisionModelTest, PrecomputeUpdatesWithPose) {
    OBBCollisionModel model({1.0, 1.0, 1.0});

    // 位姿1：OBB 在原点
    Pose pose1 = Pose::Identity();
    model.precompute(pose1);
    auto at_origin = [](const Position3D& p) { return p.norm() < 0.6; };
    EXPECT_TRUE(model.checkCollision(at_origin));

    // 位姿2：机器人平移到 (10, 0, 0)，OBB 中心也移到 (10, 0, 0)
    Pose pose2 = Pose::Identity();
    pose2.translation() = Vector3D(10.0, 0.0, 0.0);
    model.precompute(pose2);

    // 原点附近不再有碰撞
    EXPECT_FALSE(model.checkCollision(at_origin));

    // x=10 附近有碰撞
    auto at_new_pos = [](const Position3D& p) { return (p - Vector3D(10, 0, 0)).norm() < 0.6; };
    EXPECT_TRUE(model.checkCollision(at_new_pos));
}

// ─── 10. 边界补全：角点 (max_u, max_v) 确实在缓存中 ─────────────────
// precompute 中专门补全了三种边点，若漏掉会导致 OBB 棱/角漏检。
// OBB {1,1,1}，某面上角点为 (±0.5, ±0.5, ±0.5)，验证这些点存在。
TEST(OBBCollisionModelTest, BorderCompletionCornerPointsPresent) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 精确命中 OBB 某个角点 (0.5, 0.5, 0.5)
    auto corner_rule = [](const Position3D& p) {
        return (p - Vector3D(0.5, 0.5, 0.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(corner_rule))
        << "Corner point (0.5,0.5,0.5) must be in cached_points_";

    // 精确命中另一角点 (-0.5, 0.5, -0.5)
    auto corner_rule2 = [](const Position3D& p) {
        return (p - Vector3D(-0.5, 0.5, -0.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(corner_rule2))
        << "Corner point (-0.5,0.5,-0.5) must be in cached_points_";
}

// ─── 11. 非对称 OBB：三轴完全不同 ───────────────────────────────────
// 使用 {0.5, 2.0, 3.0}，验证各轴表面范围均正确。
TEST(OBBCollisionModelTest, AsymmetricOBBSurfaceBounds) {
    // half: x=0.25, y=1.0, z=1.5
    OBBCollisionModel model({0.5, 2.0, 3.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // X 面：x = ±0.25，OBB 内 y∈[-1,1], z∈[-1.5,1.5]
    auto x_face = [](const Position3D& p) { return std::abs(std::abs(p.x()) - 0.25) < 1e-9; };
    EXPECT_TRUE(model.checkCollision(x_face));

    // X 方向不应有超出 ±0.25 的采样点
    auto x_out = [](const Position3D& p) { return std::abs(p.x()) > 0.26; };
    EXPECT_FALSE(model.checkCollision(x_out));

    // Y 面：y = ±1.0
    auto y_face = [](const Position3D& p) { return std::abs(std::abs(p.y()) - 1.0) < 1e-9; };
    EXPECT_TRUE(model.checkCollision(y_face));

    // Z 面角点：(±0.25, ±1.0, ±1.5) 必须存在
    auto z_corner = [](const Position3D& p) {
        return (p - Vector3D(0.25, 1.0, 1.5)).norm() < 1e-9;
    };
    EXPECT_TRUE(model.checkCollision(z_corner));
}

// ─── 12. 旋转 + 平移组合位姿 ─────────────────────────────────────────
// 机器人位姿：绕 Z 旋转 90° 后向 X 方向平移 5m。
// OBB {2,1,1}，旋转后长轴变 Y，中心移到 (5,0,0)。
TEST(OBBCollisionModelTest, CombinedRotationAndTranslation) {
    OBBCollisionModel model({2.0, 1.0, 1.0});

    Pose robot_pose = Pose::Identity();
    robot_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Vector3D::UnitZ()).toRotationMatrix();
    robot_pose.translation() = Vector3D(5.0, 0.0, 0.0);
    model.precompute(robot_pose);

    // OBB 中心在 (5,0,0)，旋转后长轴沿 Y，所以 Y 表面在 y = ±1.0（相对中心）
    // 世界系表面点应在 (5, ±1.0, *) 附近
    auto y_surface_world = [](const Position3D& p) {
        return std::abs(p.x() - 5.0) < 1e-6 && std::abs(std::abs(p.y()) - 1.0) < 1e-6;
    };
    EXPECT_TRUE(model.checkCollision(y_surface_world))
        << "After 90deg rotation + translation, Y-face should appear at world (5, ±1, z)";

    // 原点附近不应有任何采样点
    auto near_origin = [](const Position3D& p) { return p.norm() < 3.5; };
    EXPECT_FALSE(model.checkCollision(near_origin))
        << "OBB translated to x=5 should not have points near origin";
}

// ─── 13. 缓存点数量的确定性：precompute 调用两次结果相同 ─────────────
// 验证 precompute 内部正确调用了 clear()，不会发生累积追加。
// 方法：用计数 lambda 统计遍历点数，两次 precompute 后点数应相同。
TEST(OBBCollisionModelTest, CacheCountIsDeterministic) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 第一次 precompute
    model.precompute(pose);
    int count1 = 0;
    model.checkCollision([&count1](const Position3D&) { ++count1; return false; });

    // 第二次 precompute（相同位姿）
    model.precompute(pose);
    int count2 = 0;
    model.checkCollision([&count2](const Position3D&) { ++count2; return false; });

    EXPECT_EQ(count1, count2)
        << "precompute called twice should produce same number of cached points, "
        << "got " << count1 << " vs " << count2;
    EXPECT_GT(count1, 0) << "cached_points_ should not be empty after precompute";
}

// ═══════════════════════════════════════════════════════════════════════════
// checkCost 测试
// ═══════════════════════════════════════════════════════════════════════════

// ─── 14. 未调用 precompute 直接 checkCost 抛出异常 ──────────────────────
TEST(OBBCollisionModelTest, CheckCostBeforePrecomputeThrows) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    auto cost_rule = [](const Position3D&) -> int { return 10; };
    EXPECT_THROW(model.checkCost(cost_rule), std::logic_error);
}

// ─── 15. 所有点代价相同时返回该代价 ──────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostUniformCost) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    auto uniform_cost = [](const Position3D&) -> int { return 42; };
    int max_cost = model.checkCost(pose, uniform_cost);
    EXPECT_EQ(max_cost, 42);
}

// ─── 16. 返回最大代价值 ──────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostReturnsMaximum) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 根据 z 坐标计算代价，z 越大代价越高
    auto z_based_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.z() * 100);
    };

    int max_cost = model.checkCost(pose, z_based_cost);
    // OBB {1,1,1} 的 z 范围是 [-0.5, 0.5]，最大代价应该是 50
    EXPECT_EQ(max_cost, 50);
}

// ─── 17. 处理负代价值 ────────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostHandlesNegativeValues) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    // 所有点返回负代价
    auto negative_cost = [](const Position3D&) -> int { return -10; };
    int max_cost = model.checkCost(pose, negative_cost);
    EXPECT_EQ(max_cost, 0) << "With all negative costs, should return 0 (initial value)";
}

// ─── 18. 高频接口：precompute 后多次 checkCost ───────────────────────────
TEST(OBBCollisionModelTest, CheckCostHighFreqInterfaceReusesCache) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();
    model.precompute(pose);

    // 多次调用不同的代价规则
    auto cost1 = [](const Position3D& pt) -> int { return static_cast<int>(pt.x() * 100); };
    auto cost2 = [](const Position3D& pt) -> int { return static_cast<int>(pt.y() * 100); };

    int max1 = model.checkCost(cost1);
    int max2 = model.checkCost(cost2);

    EXPECT_EQ(max1, 50);  // x 最大值 0.5
    EXPECT_EQ(max2, 50);  // y 最大值 0.5
}

// ─── 19. 基于位置的代价计算 ──────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostPositionBased) {
    OBBCollisionModel model({2.0, 2.0, 2.0});
    Pose pose = Pose::Identity();

    // 距离原点越远代价越高
    auto distance_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.norm() * 100);
    };

    int max_cost = model.checkCost(pose, distance_cost);
    // OBB {2,2,2} 的角点距离原点最远，约为 sqrt(3) ≈ 1.732
    EXPECT_GE(max_cost, 170);
    EXPECT_LE(max_cost, 174);
}

// ─── 20. 移动后代价计算更新 ──────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostUpdatesWithPose) {
    OBBCollisionModel model({1.0, 1.0, 1.0});

    // 位姿1：OBB 在原点
    Pose pose1 = Pose::Identity();
    auto x_cost = [](const Position3D& pt) -> int { return static_cast<int>(pt.x() * 100); };
    int cost1 = model.checkCost(pose1, x_cost);
    EXPECT_EQ(cost1, 50);  // x 最大值 0.5

    // 位姿2：机器人平移到 (10, 0, 0)
    Pose pose2 = Pose::Identity();
    pose2.translation() = Vector3D(10.0, 0.0, 0.0);
    int cost2 = model.checkCost(pose2, x_cost);
    EXPECT_EQ(cost2, 1050);  // x 最大值 10.5
}

// ─── 21. 捕获外部变量的代价规则 ──────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostWithCapturedVariables) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    double scale_factor = 200.0;
    auto scaled_cost = [scale_factor](const Position3D& pt) -> int {
        return static_cast<int>(pt.z() * scale_factor);
    };

    int max_cost = model.checkCost(pose, scaled_cost);
    EXPECT_EQ(max_cost, 100);  // z 最大值 0.5 * 200
}

// ─── 22. 零代价情况 ──────────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostAllZero) {
    OBBCollisionModel model({1.0, 1.0, 1.0});
    Pose pose = Pose::Identity();

    auto zero_cost = [](const Position3D&) -> int { return 0; };
    int max_cost = model.checkCost(pose, zero_cost);
    EXPECT_EQ(max_cost, 0);
}

// ─── 23. 混合正负代价值 ──────────────────────────────────────────────────
TEST(OBBCollisionModelTest, CheckCostMixedPositiveNegative) {
    OBBCollisionModel model({2.0, 2.0, 2.0});
    Pose pose = Pose::Identity();

    // x < 0 返回负值，x >= 0 返回正值
    auto mixed_cost = [](const Position3D& pt) -> int {
        return static_cast<int>(pt.x() * 100);
    };

    int max_cost = model.checkCost(pose, mixed_cost);
    EXPECT_EQ(max_cost, 100);  // x 最大值 1.0
}
```

---

### 📄 finenav_core/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_core) # Header-only

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(ros_dependencies
        eigen3_cmake_module
        rclcpp
        rclcpp_action
        tf2_ros
        tf2_eigen
        finenav_msgs
        sensor_msgs
        geometry_msgs
        finenav_util
)

# Generate parameter library for NavStateManager — moved to finenav_navigator

install(DIRECTORY include/
        DESTINATION include/
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_link_libraries(${PROJECT_NAME} INTERFACE
        fmt::fmt
        Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
        ${ros_dependencies}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        INCLUDES DESTINATION include
)
ament_export_dependencies(
        fmt
        Eigen3
        generate_parameter_library
        ${ros_dependencies}
)

ament_package()
```

---

### 📄 finenav_core/include/finenav_core/concepts.hpp

```cpp
// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <concepts>
#include <optional>
#include <utility>
#include <Eigen/Dense>

#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_core/plan/data_packet.hpp"

namespace finenav {


using Position3D = Eigen::Vector3d;
using Region3D = Eigen::AlignedBox3d;

/**
 * @brief MapConcept
 * @details
 * Map 被定义为一个具有视窗机制的空间数据容器，M: Postion3D -> DataType
 * * 在外部视角下，核心特性：
 * * 视窗机制: 仅允许操作以当前中心的矩形范围内的有效数据，视窗外的数据被视为失效（视窗可以被定义为无穷大）
 */
template <typename T>
concept MapConcept =
    std::default_initializable<T> &&
    std::movable<T> &&
    requires(T map, const Position3D& pos, const typename T::ConfigType& config, const typename T::DataType& data) {
    // --- 基本配置 ---
    typename T::ConfigType;
    typename T::DataType;
    { map.configure(config) } -> std::same_as<void>; // 配置地图参数

    // --- 视窗操作 ---
    { std::as_const(map).isInside(pos) } -> std::same_as<bool>; // 检查是否在当前视窗内
    { std::as_const(map).getWindowBounds() } -> std::convertible_to<Region3D>; // 获取当前视窗范围
    { std::as_const(map).getWindowCenter() } -> std::convertible_to<Position3D>; // 获取当前视窗中心
    { map.shiftWindowTo(pos) } -> std::same_as<void>;
    };

/**
 * @brief PlanningContext (规划上下文)
 * @details 包含算法在"这一帧"规划所需的所有不可变数据。
 *
 * robot_state 由框架从外部 RobotStateProvider 注入，算法开发者无需关心
 * finenav_localizer 内部的数学表示（NavState<S>），直接使用 geometry_msgs 组合体即可。
 */
template <typename PlannerT>
struct PlanningContext {
    finenav_msgs::msg::RobotState robot_state; ///< 当前机器人状态（pose / twist / accel）
    geometry_msgs::msg::Pose goal_pose;
    InputDataT<PlannerT> ref_traj; // TODO: 验证空轨迹的逻辑
};

/**
 * @brief PlannerConcept TODO：是否考虑改名为 Motion Generator
 * @details
 * Planner 被定义为一个访问地图试图并生成轨迹的算法模块，P: (State, Reference, Map) -> Trajectory
 * * 在外部视角下，核心特性：
 * 1. 特质自省： 暴露它的类型特质，以区分是否需要参考轨迹
 * 2. 不持有地图，只接受地图试图 MapView
 */
template <typename T>
concept PlannerConcept = std::default_initializable<T> &&
    requires {
        typename T::ConfigType;
        typename T::IMapView;
        typename InputDataT<T>;
        typename OutputDataT<T>;
    } &&
    requires(T planner, const typename T::ConfigType& config, const PlanningContext<T>& ctx, const typename T::IMapView& map_view) {

        // --- 基本配置 ---
        { planner.configure(config) } -> std::same_as<void>;

        // --- 重置接口（Episode 边界，由框架在新目标到来时调用）---
        { planner.reset() } -> std::same_as<void>;

        // --- 规划接口 ---
        { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputDataT<T>>>;
};


}
```

---

### 📄 finenav_core/include/finenav_core/map/locked_map_view.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include <shared_mutex>
#include "finenav_core/concepts.hpp"
namespace finenav {
/**
 * @brief LockedMapRO — 持有共享读锁的地图 RAII 视图。
 *
 * 构造时自动加读锁，析构时自动释放，保证 plan() 调用期间地图不被写入。
 */
template <MapConcept MapT>
class LockedMapRO {
public:
    LockedMapRO(const MapT& map, std::shared_mutex& mutex)
        : map_(map), lock_(mutex) {}
    const MapT* operator->() const { return &map_; }
    const MapT& get() const { return map_; }
private:
    const MapT& map_;
    std::shared_lock<std::shared_mutex> lock_;
};
/**
 * @brief LockedMapRW — 持有独占写锁的地图 RAII 视图。
 *
 * 构造时自动加写锁，析构时自动释放，用于主动且即时地修改地图。
 */
template <MapConcept MapT>
class LockedMapRW {
public:
    LockedMapRW(MapT& map, std::shared_mutex& mutex)
        : map_(map), lock_(mutex) {}
    MapT* operator->() { return &map_; }
    MapT& get() { return map_; }
private:
    MapT& map_;
    std::unique_lock<std::shared_mutex> lock_;
};
} // namespace finenav
```

---

### 📄 finenav_core/include/finenav_core/map/map_server.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_core/concepts.hpp"
#include "finenav_core/map/locked_map_view.hpp"
#include "finenav_core/map/update_source.hpp"

namespace finenav {

// ==============================================================================
// MapServer Declaration
// ==============================================================================

/**
 * @brief MapServer — 管理一张地图实例的生命周期、并发访问与周期性更新。
 *
 * - 通过 addUpdateSource() 注册数据源，由内部定时器定期将缓存消息融合进地图。
 * - 通过 getLockedReadView() / getLockedWriteView() 获取 RAII 锁视图，保证线程安全。
 * - 可选接入 FineNavLocalizer 实现地图窗口跟随机器人位移。
 *
 * @tparam MapT  地图类型，必须满足 MapConcept。
 */
template <MapConcept MapT>
class MapServer {
public:
    /**
     * @brief 单个数据源的缓存与超时策略。
     */
    struct SourceBufferPolicy {
        std::size_t max_buffer_size{32};       ///< 最大缓存帧数（0 = 无限制）
        double observation_keep_time_sec{0.0}; ///< 消息最大保留时长（0 = 永不过期）
        double source_timeout_sec{0.0};        ///< 数据源超时阈值（0 = 禁用）
    };

    using PreUpdateHook  = std::function<void(MapT& map)>;
    using PostUpdateHook = std::function<void(const MapT& map)>;

    template <typename MsgT>
    using Fuser = std::function<void(const MsgT& msg, const rclcpp::Time& stamp, MapT& map)>;

    /**
     * @brief 构造 MapServer，初始化内部定时器以定期执行地图更新。
     * @param node           ROS 2 节点指针
     * @param server_name    MapServer 的名称（同时作为私有子节点名）
     * @param update_rate_hz 传感器融合定时器频率（Hz），默认 10 Hz
     * @param localizer      可选的定位器指针；非空时启动地图位移定时器
     * @param shift_rate_hz  地图位移定时器频率（Hz），默认 50 Hz
     */
    explicit MapServer(
        rclcpp::Node::SharedPtr node,
        const std::string& server_name,
        double update_rate_hz = 10.0,
        FineNavLocalizer* localizer = nullptr,
        double shift_rate_hz = 50.0);

    /** @brief 注册在每次地图更新周期**前**执行的回调。 */
    void registerPreUpdateHook(std::function<void(MapT& map)> hook);

    /** @brief 注册在每次地图更新周期**后**执行的回调。 */
    void registerPostUpdateHook(std::function<void(MapT& map)> hook);

    /**
     * @brief 获取 RAII 读视图（共享锁）。
     * @return 受读锁保护的 LockedMapRO<MapT>
     */
    LockedMapRO<MapT> getLockedReadView();

    /**
     * @brief 获取 RAII 读写视图（独占锁）。用于主动即时修改地图。
     * @return 受写锁保护的 LockedMapRW<MapT>
     */
    LockedMapRW<MapT> getLockedWriteView();

    /**
     * @brief 注册一个数据源，返回线程安全的推送闭包。
     *
     * 返回的闭包可从任意线程调用，数据将被缓存至内部队列，
     * 并在下次 runUpdateCycle() 时由 fuser 融合进地图。
     *
     * @tparam MsgT       消息类型
     * @param source_name 数据源名称（用于日志）
     * @param policy      缓存与超时策略
     * @param fuser       融合函数：(const MsgT&, MapT&) -> void
     * @return 线程安全的推送闭包：void(MsgT&&, const rclcpp::Time&)
     */
    template <typename MsgT>
    std::function<void(MsgT&&, const rclcpp::Time&)> addUpdateSource(
        const std::string& source_name, SourceBufferPolicy policy, Fuser<MsgT> fuser);

private:
    void runUpdateCycle(const rclcpp::Time& now);
    void runShiftCycle(const rclcpp::Time& now);

    template <typename T>
    void keep_alive(std::shared_ptr<T> handle);

    // ---- ROS infrastructure ----
    rclcpp::Node::SharedPtr     node_;
    std::string                 server_name_;
    rclcpp::Node::SharedPtr     pnode_;
    std::unique_ptr<NodeThread> pnode_thread_;

    // ---- Map data & lock ----
    std::unique_ptr<MapT> map_instance_;
    std::shared_mutex     rw_mutex_;

    // ---- Optional localizer for map shifting ----
    FineNavLocalizer*            localizer_{nullptr};
    rclcpp::TimerBase::SharedPtr shift_timer_;

    // ---- Hooks, sources & timers ----
    std::vector<std::function<void(MapT&)>>          pre_update_hooks_;
    std::vector<std::function<void(MapT&)>>          post_update_hooks_;
    std::vector<std::shared_ptr<IUpdateSource<MapT>>> update_sources_;
    std::vector<std::shared_ptr<void>>               held_param_handles_;
    rclcpp::TimerBase::SharedPtr                     update_timer_;
};

}  // namespace finenav

#include "finenav_core/map/map_server_impl.hpp"

```

---

### 📄 finenav_core/include/finenav_core/map/map_server_impl.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
// This file is included at the bottom of map_server.hpp.
// It contains out-of-line template method implementations for MapServer.
// Do NOT include this file directly — include map_server.hpp instead.
#pragma once
namespace finenav {
// ==============================================================================
// Constructor
// ==============================================================================
template <MapConcept MapT>
MapServer<MapT>::MapServer(
    rclcpp::Node::SharedPtr node,
    const std::string& server_name,
    double update_rate_hz,
    FineNavLocalizer* localizer,
    double shift_rate_hz)
    : node_(node), server_name_(server_name), localizer_(localizer)
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(server_name_, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);
    map_instance_ = std::make_unique<MapT>();
    keep_alive(AlgoConfigurator<MapT>::load(pnode_, *map_instance_));
    // 传感器观测更新定时器
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { this->runUpdateCycle(this->pnode_->now()); });
    // 地图窗口位移定时器（可选）
    if (localizer_ != nullptr) {
        auto shift_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / shift_rate_hz));
        shift_timer_ = rclcpp::create_timer(
            pnode_, pnode_->get_clock(), shift_period,
            [this]() { this->runShiftCycle(this->pnode_->now()); });
    }
}
// ==============================================================================
// Hook Registration
// ==============================================================================
template <MapConcept MapT>
void MapServer<MapT>::registerPreUpdateHook(std::function<void(MapT& map)> hook) {
    pre_update_hooks_.push_back(std::move(hook));
}
template <MapConcept MapT>
void MapServer<MapT>::registerPostUpdateHook(std::function<void(MapT& map)> hook) {
    post_update_hooks_.push_back(std::move(hook));
}
// ==============================================================================
// Locked View Accessors
// ==============================================================================
template <MapConcept MapT>
LockedMapRO<MapT> MapServer<MapT>::getLockedReadView() {
    return LockedMapRO<MapT>(*map_instance_, rw_mutex_);
}
template <MapConcept MapT>
LockedMapRW<MapT> MapServer<MapT>::getLockedWriteView() {
    return LockedMapRW<MapT>(*map_instance_, rw_mutex_);
}
// ==============================================================================
// addUpdateSource
// ==============================================================================
template <MapConcept MapT>
template <typename MsgT>
std::function<void(MsgT&&, const rclcpp::Time&)>
MapServer<MapT>::addUpdateSource(
    const std::string& source_name,
    SourceBufferPolicy policy,
    Fuser<MsgT> fuser)
{
    using Source = UpdateSource<MapT, MsgT>;
    typename Source::SourceBufferPolicy src_policy{
        policy.max_buffer_size,
        policy.observation_keep_time_sec,
        policy.source_timeout_sec};
    auto source = std::make_shared<Source>(source_name, src_policy, std::move(fuser));
    update_sources_.push_back(source);
    RCLCPP_DEBUG_STREAM(node_->get_logger(),
        "MapServer added update source: " << source_name);
    std::weak_ptr<Source> weak_source = source;
    return [weak_source](MsgT&& msg, const rclcpp::Time& stamp) {
        if (auto src = weak_source.lock()) {
            src->enqueueThreadSafe(std::move(msg), stamp);
        }
    };
}
// ==============================================================================
// Internal Update Cycles
// ==============================================================================
template <MapConcept MapT>
void MapServer<MapT>::runUpdateCycle(const rclcpp::Time& now) {
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    for (auto& hook : pre_update_hooks_)  hook(*map_instance_);
    for (auto& src  : update_sources_)   src->applyBuffered(*map_instance_, now);
    for (auto& hook : post_update_hooks_) hook(*map_instance_);
}
template <MapConcept MapT>
void MapServer<MapT>::runShiftCycle(const rclcpp::Time& now) {
    auto latest_pose = localizer_->getState().p;
    auto latest_time = localizer_->getLastUpdateTime();
    // TODO: 必须有一个原子性获取时间+状态的接口，否则存在数据竞争风险
    std::unique_lock<std::shared_mutex> lock(rw_mutex_);
    map_instance_->shiftWindowTo(latest_pose);
    RCLCPP_DEBUG_STREAM(pnode_->get_logger(),
        "MapServer shifted with pose delayed by "
        << (now - latest_time).seconds() << " seconds");
}
// ==============================================================================
// Utility
// ==============================================================================
template <MapConcept MapT>
template <typename T>
void MapServer<MapT>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) held_param_handles_.push_back(handle);
}
} // namespace finenav
```

---

### 📄 finenav_core/include/finenav_core/map/update_source.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "finenav_core/concepts.hpp"
namespace finenav {
// Forward declaration — UpdateSource is owned by MapServer.
template <MapConcept MapT>
class MapServer;
// ==============================================================================
// IUpdateSource — type-erased interface for MapServer's source list
// ==============================================================================
template <MapConcept MapT>
class IUpdateSource {
public:
    virtual ~IUpdateSource() = default;
    virtual void applyBuffered(MapT& map, const rclcpp::Time& now) = 0;
};
// ==============================================================================
// UpdateSource — typed concrete source, holds buffered messages and a fuser
// ==============================================================================
/**
 * @brief UpdateSource — 持有单个数据源的缓存队列与融合函数。
 *
 * 线程安全：enqueueThreadSafe() 可从任意线程调用；
 * applyBuffered() 由 MapServer 的更新定时器线程调用（持有写锁）。
 *
 * @tparam MapT  目标地图类型
 * @tparam MsgT  输入消息类型
 */
template <MapConcept MapT, typename MsgT>
class UpdateSource final : public IUpdateSource<MapT> {
public:
    struct SourceBufferPolicy {
        std::size_t max_buffer_size{32};
        double observation_keep_time_sec{0.0};
        double source_timeout_sec{0.0};
    };
    using Fuser = std::function<void(const MsgT& msg, const rclcpp::Time& stamp, MapT& map)>;
    UpdateSource(const std::string& source_name, SourceBufferPolicy policy, Fuser fuser);
    /** Thread-safe: 从任意线程推入一条消息。 */
    void enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp);
    /** 在写锁保护下，将缓存消息全部融合进地图。 */
    void applyBuffered(MapT& map, const rclcpp::Time& now) override;
private:
    struct BufferedMessage {
        rclcpp::Time stamp;
        MsgT msg;
    };
    void enqueue(MsgT&& msg, const rclcpp::Time& stamp);
    void pruneExpired(const rclcpp::Time& now);
    std::string source_name_;
    SourceBufferPolicy policy_;
    Fuser fuser_;
    std::mutex mutex_;
    std::deque<BufferedMessage> buffer_;
    rclcpp::Time last_seen_stamp_{0, 0, RCL_ROS_TIME};
    bool has_seen_message_{false};
};
// ==============================================================================
// UpdateSource Implementation
// ==============================================================================
template <MapConcept MapT, typename MsgT>
UpdateSource<MapT, MsgT>::UpdateSource(
    const std::string& source_name, SourceBufferPolicy policy, Fuser fuser)
    : source_name_(source_name), policy_(policy), fuser_(std::move(fuser)) {}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::enqueueThreadSafe(MsgT&& msg, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    enqueue(std::move(msg), stamp);
}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::applyBuffered(MapT& map, const rclcpp::Time& now) {
    std::deque<BufferedMessage> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pruneExpired(now);
        if (policy_.source_timeout_sec > 0.0 && has_seen_message_) {
            const auto elapsed = (now - last_seen_stamp_).seconds();
            if (elapsed > policy_.source_timeout_sec) {
                if (!buffer_.empty()) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("MapServer"),
                        "UpdateSource '" << source_name_ << "' has timed out ("
                        << elapsed << "s). Buffer cleared!");
                }
                buffer_.clear();
            }
        }
        pending.swap(buffer_);
    }
    if (!fuser_) return;
    if (!pending.empty()) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MapServer"),
            "UpdateSource '" << source_name_ << "' applying "
            << pending.size() << " buffered messages.");
    }
    for (const auto& item : pending) {
        fuser_(item.msg, item.stamp, map);
    }
}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::enqueue(MsgT&& msg, const rclcpp::Time& stamp) {
    if (policy_.max_buffer_size > 0 && buffer_.size() >= policy_.max_buffer_size) {
        buffer_.pop_front();
    }
    buffer_.push_back(BufferedMessage{stamp, std::move(msg)});
    last_seen_stamp_  = stamp;
    has_seen_message_ = true;
}
template <MapConcept MapT, typename MsgT>
void UpdateSource<MapT, MsgT>::pruneExpired(const rclcpp::Time& now) {
    if (policy_.observation_keep_time_sec <= 0.0) return;
    while (!buffer_.empty()) {
        if ((now - buffer_.front().stamp).seconds() <= policy_.observation_keep_time_sec) break;
        buffer_.pop_front();
    }
}
} // namespace finenav
```

---

### 📄 finenav_core/include/finenav_core/plan/control_layer.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_core/concepts.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/plan_context.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/planner_set.hpp"
#include "finenav_core/plan/ros_conversions.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/simple_action_server.hpp"
#include "finenav_util/node_thread.hpp"

namespace finenav {

// ==============================================================================
// ControlLayer Declaration
// ==============================================================================

/**
 * @brief ControlLayer — 将 Planner、MapView 与执行策略绑定为一个规划层。
 *
 * @tparam PlannersT      PlannerSet<...>，枚举所有支持的规划器类型。
 * @tparam ExecutionPolicy  执行策略类型，默认 SingleShotPolicy。
 *                          支持扩展：新增策略只需定义 policy struct + 添加对应 executeImpl() 重载。
 */
template <typename PlannersT, typename ExecutionPolicy = SingleShotPolicy>
class ControlLayer {
public:
    using MapViewsVariant = GenerateMapViewsVariant<PlannersT>;
    using ActionT         = finenav_msgs::action::ComputePlan;
    using ActionServer    = SimpleActionServer<ActionT>;

    /**
     * @brief BoundMapView — 地图视图 + RAII 读锁的聚合体。
     *
     * lock_guard 持有 LockedMapRO<MapT> 的 shared_ptr，保证在 planner.plan() 整个
     * 调用期间读锁不被释放；view_variant 是规划器实际读取的视图指针。
     */
    struct BoundMapView {
        MapViewsVariant view_variant;
        std::shared_ptr<void> lock_guard; ///< 持有 LockedMapRO<MapT>，离开作用域后自动释放读锁
    };

    using MapViewFactory      = std::function<BoundMapView()>;
    using PrePlanCallback     = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback    = std::function<bool(PostPlanContext&)>;
    using RobotStateProvider  = std::function<finenav_msgs::msg::RobotState()>;

    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& layer_name,
        ExecutionPolicy policy = ExecutionPolicy{});

    void setInputTopic(const std::string& topic);
    void setOutputTopic(const std::string& topic);
    std::string getName() const;
    void setupDataPorts();

    /**
     * @brief 将 MapServer 与规划器所需的地图视图绑定。
     *
     * 框架在每次调用 planner.plan() 前自动通过 server 申请读锁，并将加锁后的
     * const MapT& 传给 builder 以构造 IMapView；读锁在 plan() 返回后自动释放。
     *
     * @tparam MapT      地图类型，从 server 参数自动推导，必须满足 MapConcept
     * @tparam BuilderFn 任意可调用类型，签名：(const MapT&) -> MapViewsVariant-compatible
     * @param server     提供地图数据的 MapServer（弱引用持有）
     * @param builder    适配函数：接收加锁的 const MapT&，返回可构造 MapViewsVariant 的值
     */
    template <MapConcept MapT, typename BuilderFn>
    requires std::is_invocable_v<BuilderFn, const MapT&> &&
             std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>, MapViewsVariant>
    void bind_map_view(std::shared_ptr<MapServer<MapT>> server, BuilderFn&& builder);

    void set_on_pre_plan(PrePlanCallback cb)               { on_pre_plan_           = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb)             { on_post_plan_          = std::move(cb); }
    void set_robot_state_provider(RobotStateProvider prov) { robot_state_provider_  = std::move(prov); }


    const std::string& get_active_planner_id() const { return active_planner_id_; }

protected:
    MapViewFactory      map_view_factory_;
    PrePlanCallback     on_pre_plan_;
    PostPlanCallback    on_post_plan_;
    RobotStateProvider  robot_state_provider_;

    rclcpp::Node::SharedPtr      node_;
    rclcpp::Node::SharedPtr      pnode_;
    std::unique_ptr<NodeThread>  pnode_thread_;

    typename PlannersT::Tuple planners_instance_;

    std::string     active_planner_id_;
    ExecutionPolicy policy_;

private:
    // ---- Utilities ----
    template <typename T>
    void keep_alive(std::shared_ptr<T> handle);

    void PlanRefCallback(const finenav_msgs::msg::Trajectory& msg);

    // ---- Episode entry-point ----
    void execute();

    // ---- Policy dispatch — add new overload to support new execution mode ----
    void executeImpl(SingleShotPolicy& policy);
    void executeImpl(TrackingPolicy&   policy);

    // ---- Shared helpers (used by both policies) ----

    /** @brief 对匹配 active_planner_id_ 的规划器调用 reset()，在 Episode 边界处触发。 */
    void resetActivePlanner();

    /**
     * @brief 执行一次完整规划步骤：pre-cb → ctx → 地图锁 → plan() → post-cb → 发布。
     * @return true 表示规划成功并发布了结果。
     */
    bool planOnce(geometry_msgs::msg::Pose goal_pose);

    // ---- Data ----
    std::string layer_name_;
    std::string input_topic_;
    std::string output_topic_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<ActionServer> action_server_;

    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr plan_ref_sub_;
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr    plan_result_pub_;

    std::mutex                       plan_ref_mutex_;
    finenav_msgs::msg::Trajectory    plan_ref_msg_;

    std::vector<std::shared_ptr<void>> param_handles_; ///< 防止参数回调句柄被析构
};

} // namespace finenav

#include "finenav_core/plan/control_layer_impl.hpp"
```

---

### 📄 finenav_core/include/finenav_core/plan/control_layer_impl.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of control_layer.hpp.
// It contains the out-of-line template method implementations for ControlLayer.
// Do NOT include this file directly — include control_layer.hpp instead.

#pragma once

namespace finenav {

// ==============================================================================
// Constructor
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
ControlLayer<PlannersT, ExecutionPolicy>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& layer_name,
    ExecutionPolicy policy)
    : node_(node), layer_name_(layer_name), policy_(std::move(policy))
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    pnode_->declare_parameter<std::string>("example_param", "hello, world");

    if (!layer_name.empty()) {
        action_server_ = std::make_unique<ActionServer>(
            pnode_,
            layer_name,
            std::bind(&ControlLayer::execute, this));
    }

    std::apply([this](auto&... planners) {
        ([this](auto& p) {
            using T = std::decay_t<decltype(p)>;
            RCLCPP_INFO_STREAM(pnode_->get_logger(), "Loading planner: " << AlgoConfigurator<T>::name);
            keep_alive(AlgoConfigurator<T>::load(pnode_, p));
        }(planners), ...);
    }, planners_instance_);
}

// ==============================================================================
// Topic / Port Setup
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setOutputTopic(const std::string& topic) {
    output_topic_ = topic;
}

template <typename PlannersT, typename ExecutionPolicy>
std::string ControlLayer<PlannersT, ExecutionPolicy>::getName() const {
    return pnode_->get_name();
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setupDataPorts() {
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_msgs::msg::Trajectory>(
            input_topic_, 10,
            std::bind(&ControlLayer::PlanRefCallback, this, std::placeholders::_1));
    }
    if (!output_topic_.empty()) {
        plan_result_pub_ = pnode_->create_publisher<finenav_msgs::msg::Trajectory>(output_topic_, 10);
    }
}

// ==============================================================================
// Utilities
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
template <typename T>
void ControlLayer<PlannersT, ExecutionPolicy>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) {
        param_handles_.push_back(handle);
    }
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::PlanRefCallback(
    const finenav_msgs::msg::Trajectory& msg)
{
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = msg;
}

// ==============================================================================
// Shared Helpers
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::resetActivePlanner() {
    std::apply([&](auto&... planners) {
        ([&](auto& p) {
            using PlannerT = std::decay_t<decltype(p)>;
            if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {
                if constexpr (requires { p.reset(); }) {
                    p.reset();
                    RCLCPP_INFO(pnode_->get_logger(),
                        "Planner '%s' reset at episode boundary.", active_planner_id_.c_str());
                }
            }
        }(planners), ...);
    }, planners_instance_);
}

template <typename PlannersT, typename ExecutionPolicy>
bool ControlLayer<PlannersT, ExecutionPolicy>::planOnce(geometry_msgs::msg::Pose goal_pose) {
    if (!map_view_factory_) {
        RCLCPP_ERROR(pnode_->get_logger(), "MapViewFactory not set!");
        return false;
    }
    if (!robot_state_provider_) {
        RCLCPP_ERROR(pnode_->get_logger(),
            "RobotStateProvider not set! Call set_robot_state_provider() first.");
        return false;
    }

    finenav_msgs::msg::RobotState current_robot_state = robot_state_provider_();

    finenav_msgs::msg::Trajectory current_ref_msg;
    {
        std::lock_guard<std::mutex> lock(plan_ref_mutex_);
        current_ref_msg = plan_ref_msg_;
    }

    bool plan_success  = false;
    bool planner_found = false;
    finenav_msgs::msg::Trajectory output_traj_msg;

    auto try_run_planner = [&](auto& planner) -> bool {
        using PlannerT = std::decay_t<decltype(planner)>;
        if (AlgoConfigurator<PlannerT>::name != active_planner_id_) {
            return false;
        }

        auto opt_ctx = [&]() -> std::optional<PlanningContext<PlannerT>> {
            PrePlanContext pre_ctx{
                active_planner_id_, current_robot_state, goal_pose, current_ref_msg};

            if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
                return std::nullopt; // 用户拦截，中止此次规划
            }

            PlanningContext<PlannerT> ctx;
            ctx.robot_state = pre_ctx.robot_state;
            ctx.goal_pose   = pre_ctx.goal_pose;
            fromROSMsg(std::move(pre_ctx.ref_traj), ctx.ref_traj);
            return ctx;
        }();

        if (!opt_ctx.has_value()) {
            return true; // 短路遍历
        }

        auto& ctx   = opt_ctx.value();
        auto  bound = map_view_factory_();

        using ExpectedPtrType = std::shared_ptr<typename PlannerT::IMapView>;
        if (auto* ptr = std::get_if<ExpectedPtrType>(&bound.view_variant)) {
            if (*ptr) {
                // bound.lock_guard 持有读锁，plan() 调用期间地图不被写入
                auto result = planner.plan(ctx, **ptr);
                if (result.has_value()) {
                    plan_success = true;
                    toROSMsg(std::move(result.value()), output_traj_msg);
                } else {
                    RCLCPP_WARN(pnode_->get_logger(), "plan() returned nullopt.");
                }
            } else {
                RCLCPP_ERROR(pnode_->get_logger(),
                    "MapView ptr is null for planner '%s'.", active_planner_id_.c_str());
            }
        } else {
            RCLCPP_ERROR(pnode_->get_logger(),
                "MapView type mismatch for planner '%s'.", active_planner_id_.c_str());
        }

        if (on_post_plan_) {
            PostPlanContext post_ctx{active_planner_id_, plan_success, output_traj_msg};
            plan_success = on_post_plan_(post_ctx);
        }

        return true; // 短路遍历
    };

    std::apply([&](auto&... planners) {
        planner_found = (try_run_planner(planners) || ...);
    }, planners_instance_);

    if (!planner_found) {
        RCLCPP_ERROR_STREAM(pnode_->get_logger(),
            "Planner '" << active_planner_id_ << "' not found.");
        return false;
    }

    if (plan_success && plan_result_pub_) {
        plan_result_pub_->publish(output_traj_msg);
    }

    return plan_success;
}

// ==============================================================================
// execute() — Episode Entry-Point
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::execute() {
    RCLCPP_INFO(pnode_->get_logger(), "New goal received — starting episode.");

    auto goal          = action_server_->get_current_goal();
    active_planner_id_ = goal->planner_id;

    // Episode 边界：在任何 plan() 调用前重置规划器内部状态
    resetActivePlanner();

    // 按策略执行
    executeImpl(policy_);
}

// ==============================================================================
// executeImpl — SingleShotPolicy
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::executeImpl(SingleShotPolicy&) {
    auto goal_pose = action_server_->get_current_goal()->goal;

    if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(pnode_->get_logger(), "SingleShot: goal cancelled before plan.");
        action_server_->terminate_all();
        return;
    }

    if (!planOnce(goal_pose)) {
        RCLCPP_ERROR(pnode_->get_logger(), "SingleShot: plan() failed.");
        action_server_->terminate_all();
        return;
    }

    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "SingleShot: goal succeeded.");
}

// ==============================================================================
// executeImpl — TrackingPolicy
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::executeImpl(TrackingPolicy& policy) {
    auto goal_pose = action_server_->get_current_goal()->goal;
    rclcpp::Rate rate(policy.frequency_hz);

    RCLCPP_INFO(pnode_->get_logger(),
        "Tracking: control loop started at %.1f Hz.", policy.frequency_hz);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            // BT 驱动的正常取消（如 IsGoalReached/IsStuck 触发），不代表错误。
            RCLCPP_INFO(pnode_->get_logger(),
                "Tracking: goal cancelled by BT (normal termination).");
            action_server_->terminate_all();
            return;
        }

        // 检测 BT 侧通过 shouldResendGoal() 发来的新 goal（进入 pending slot）。
        // 原地切换 goal_pose，不调用 resetActivePlanner()，保留 MPPI 热启动状态。
        if (action_server_->is_preempt_requested()) {
            auto new_goal = action_server_->accept_pending_goal();
            if (new_goal) {
                goal_pose = new_goal->goal;
                RCLCPP_INFO(pnode_->get_logger(),
                    "Tracking: goal preempted in-loop (%.2f, %.2f) — MPPI warm-start preserved.",
                    goal_pose.position.x, goal_pose.position.y);
            }
        }

        if (!planOnce(goal_pose)) {
            RCLCPP_ERROR(pnode_->get_logger(), "Tracking: plan() failed — aborting.");
            action_server_->terminate_all();
            return;
        }


        rate.sleep();
    }

    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "Tracking: goal succeeded.");
}

// ==============================================================================
// bind_map_view
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
template <MapConcept MapT, typename BuilderFn>
requires std::is_invocable_v<BuilderFn, const MapT&> &&
         std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>,
                             typename ControlLayer<PlannersT, ExecutionPolicy>::MapViewsVariant>
void ControlLayer<PlannersT, ExecutionPolicy>::bind_map_view(
    std::shared_ptr<MapServer<MapT>> server,
    BuilderFn&& builder)
{
    std::function<MapViewsVariant(const MapT&)> erased_builder =
        std::forward<BuilderFn>(builder);
    std::weak_ptr<MapServer<MapT>> weak_server = server;

    map_view_factory_ = [weak_server, erased_builder = std::move(erased_builder)]() -> BoundMapView
    {
        auto srv = weak_server.lock();
        if (!srv) {
            return BoundMapView{};
        }

        auto locked = std::make_shared<LockedMapRO<MapT>>(srv->getLockedReadView());
        MapViewsVariant view_variant = erased_builder(locked->get());
        return BoundMapView{std::move(view_variant), std::move(locked)};
    };
}

} // namespace finenav

```

---

### 📄 finenav_core/include/finenav_core/plan/data_packet.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <cstdint>

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace finenav {

struct IOProfile {
    bool pose = false;
    bool vel  = false;
    bool accel= false;
    bool time = false;
};

template <bool Enable> struct PoseMixin;
template <> struct PoseMixin<true> { std::vector<geometry_msgs::msg::Pose> poses; };
template <> struct PoseMixin<false> {};

template <bool Enable> struct VelMixin;
template <> struct VelMixin<true> { std::vector<geometry_msgs::msg::Twist> twists; };
template <> struct VelMixin<false> {};

template <bool Enable> struct AccelMixin;
template <> struct AccelMixin<true> { std::vector<geometry_msgs::msg::Accel> accels; };
template <> struct AccelMixin<false> {};

template <bool Enable> struct TimeMixin;
template <> struct TimeMixin<true> { std::vector<builtin_interfaces::msg::Duration> times; };
template <> struct TimeMixin<false> {};


struct EmptyBlock {};

// 核心容器：根据 C++20 Config 对象进行多重继承
template <IOProfile Config>
struct DataPacket :
    public PoseMixin<Config.pose>,
    public VelMixin<Config.vel>,
    public AccelMixin<Config.accel>,
    public TimeMixin<Config.time>
{};

// 类型别名 helper
template <typename PlannerT>
using InputDataT = DataPacket<PlannerT::InputProfile>;

template <typename PlannerT>
using OutputDataT = DataPacket<PlannerT::OutputProfile>;



}


```

---

### 📄 finenav_core/include/finenav_core/plan/execution_policy.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
namespace finenav {
/**
 * @brief SingleShotPolicy — 每次 Action Goal 执行一次 plan()，完成后立即 succeed。
 *
 * 适用于离线规划、测试等"请求-响应"场景。
 * planner.reset() 在 execute() 开始时调用（Episode 边界）。
 */
struct SingleShotPolicy {};
/**
 * @brief TrackingPolicy — 每次 Action Goal 以固定频率循环调用 plan()，直到取消或目标到达。
 *
 * 适用于 MPPI 等需要持续追踪的在线控制器。
 * planner.reset() 仅在 execute() 开始时调用一次；循环内各帧 plan() 共享热启动状态。
 *
 * @param frequency_hz  控制循环频率（Hz），默认 10 Hz。
 */
struct TrackingPolicy {
    double frequency_hz = 10.0;
};
} // namespace finenav
```

---

### 📄 finenav_core/include/finenav_core/plan/plan_context.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <string>

namespace finenav {

/**
 * @brief PrePlanContext — 规划前回调所携带的上下文。
 *
 * 回调可修改 robot_state / goal_pose / ref_traj，
 * 也可通过返回 false 中止此次规划。
 */
struct PrePlanContext {
    const std::string& active_planner_id;            // [RO] 当前请求的规划器 ID
    finenav_msgs::msg::RobotState& robot_state;      // [RW] 当前机器人状态（可被回调修改）
    geometry_msgs::msg::Pose& goal_pose;             // [RW] 目标位姿
    finenav_msgs::msg::Trajectory& ref_traj;         // [RW] 规划参考轨迹
};

/**
 * @brief PostPlanContext — 规划后回调所携带的上下文。
 *
 * 回调可修改 result_traj，也可通过返回 false 将规划标记为失败。
 */
struct PostPlanContext {
    const std::string& active_planner_id;       // [RO] 当前请求的规划器 ID
    const bool is_success;                      // [RO] 规划结果是否成功
    finenav_msgs::msg::Trajectory& result_traj; // [RW] 规划结果轨迹
};

} // namespace finenav

```

---

### 📄 finenav_core/include/finenav_core/plan/planner_set.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <tuple>
#include <variant>
#include <memory>

namespace finenav {

// ==============================================================================
// PlannerSet — 声明一组规划器类型，供 ControlLayer 枚举
// ==============================================================================

template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

// ==============================================================================
// GenerateMapViewsVariant — 从 PlannerSet 推导地图视图 variant 类型
// ==============================================================================

template <typename PlannerT>
struct GetMapViewSharedPtr {
    using type = std::shared_ptr<typename PlannerT::IMapView>;
};

template <typename Tuple> struct ToVariant;

template <typename... Planners>
struct ToVariant<std::tuple<Planners...>> {
    using type = std::variant<typename GetMapViewSharedPtr<Planners>::type...>;
};

/**
 * @brief 从 PlannerSet 中提取每个规划器的 IMapView，生成 std::variant<shared_ptr<...>...>。
 *
 * 用于 ControlLayer::BoundMapView::view_variant 成员的类型推导。
 */
template <typename PlannerSetT>
using GenerateMapViewsVariant = typename ToVariant<typename PlannerSetT::Tuple>::type;

} // namespace finenav

```

---

### 📄 finenav_core/include/finenav_core/plan/ros_conversions.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cstdint>
#include "finenav_core/plan/data_packet.hpp"
#include "finenav_msgs/msg/trajectory.hpp"

namespace finenav {

consteval uint32_t ProfileToMsgMask(const IOProfile& p) {
    uint32_t mask = 0;
    if (p.pose)  mask |= 1;
    if (p.vel)   mask |= 2;
    if (p.accel) mask |= 4;
    if (p.time)  mask |= 8;
    return mask;
}

// TODO: func: checkConsistency

template <IOProfile Config>
void toROSMsg(const finenav::DataPacket<Config>& src,
              finenav_msgs::msg::Trajectory& dst)
{
    dst.valid_fields = ProfileToMsgMask(Config);
    if constexpr (Config.pose)  dst.poses = src.poses;
    if constexpr (Config.vel)   dst.twists = src.twists;
    if constexpr (Config.accel) dst.accels = src.accels;
    if constexpr (Config.time)  dst.time_from_start = src.times;
}

template <IOProfile Config>
void toROSMsg(finenav::DataPacket<Config>&& src,
              finenav_msgs::msg::Trajectory& dst)
{
    dst.valid_fields = ProfileToMsgMask(Config);
    if constexpr (Config.pose)  dst.poses = std::move(src.poses);
    if constexpr (Config.vel)   dst.twists = std::move(src.twists);
    if constexpr (Config.accel) dst.accels = std::move(src.accels);
    if constexpr (Config.time)  dst.time_from_start = std::move(src.times);
}

template <IOProfile Config>
void fromROSMsg(const finenav_msgs::msg::Trajectory& src,
                finenav::DataPacket<Config>& dst)
{
    uint32_t required = ProfileToMsgMask(Config);
    if ((src.valid_fields & required) != required) {
        throw std::runtime_error("Input trajectory message is missing required fields.");
    }
    if constexpr (Config.pose)  dst.poses = src.poses;
    if constexpr (Config.vel)   dst.twists = src.twists;
    if constexpr (Config.accel) dst.accels = src.accels;
    if constexpr (Config.time)  dst.times = src.time_from_start;
}

template <IOProfile Config>
void fromROSMsg(finenav_msgs::msg::Trajectory&& src,
                finenav::DataPacket<Config>& dst)
{
    uint32_t required = ProfileToMsgMask(Config);
    if ((src.valid_fields & required) != required) {
        throw std::runtime_error("Input trajectory message is missing required fields.");
    }
    if constexpr (Config.pose)  dst.poses = std::move(src.poses);
    if constexpr (Config.vel)   dst.twists = std::move(src.twists);
    if constexpr (Config.accel) dst.accels = std::move(src.accels);
    if constexpr (Config.time)  dst.times = std::move(src.time_from_start);
}

} // namespace finenav
```

---

### 📄 finenav_core/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_core</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>fmt</depend>
    <depend>eigen</depend>
    <depend>eigen3_cmake_module</depend>

    <depend>rclcpp</depend>
    <depend>rclcpp_action</depend>
    <depend>tf2_ros</depend>
    <depend>tf2_eigen</depend>

    <depend>sensor_msgs</depend>
    <depend>geometry_msgs</depend>
    <depend>finenav_msgs</depend>
    <depend>finenav_util</depend>
    <depend>generate_parameter_library</depend>

    <test_depend>ament_cmake_gtest</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_demo/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_demo)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
        rclcpp
        rclcpp_action
        finenav_engine
        finenav_map_grid_map
        finenav_mppi_controller
        finenav_core
        finenav_collision_model
        finenav_motion_decorator
        nav_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_eigen
        message_filters
        pcl_conversions
)

foreach(dep IN LISTS dependencies)
    find_package(${dep} REQUIRED)
endforeach()

add_subdirectory(stvl_with_mppi)

ament_package()
```

---

### 📄 finenav_demo/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_demo</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>rclcpp</depend>
    <depend>finenav_engine</depend>
    <depend>finenav_map_grid_map</depend>
    <depend>finenav_mppi_controller</depend>
    <depend>finenav_core</depend>
    <depend>finenav_collision_model</depend>

    <depend>pcl_conversions</depend>
    <depend>opencv</depend>

    <depend>nav_msgs</depend>
    <depend>geometry_msgs</depend>
    <depend>sensor_msgs</depend>
    <depend>tf2</depend>
    <depend>tf2_ros</depend>
    <depend>tf2_eigen</depend>
    <depend>message_filters</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_demo/stvl_with_mppi/CMakeLists.txt

```
add_library(terrain_lib
    src/terrain_analysis.cpp
    src/terrain_mapview.cpp
    src/stvl_manager.cpp
)
ament_target_dependencies(terrain_lib ${dependencies})

# terrain_mapview.hpp uses OpenCV and PCL
target_link_libraries(terrain_lib
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_include_directories(terrain_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


add_executable(hello_finenav src/hello_finenav.cpp)
target_link_libraries(hello_finenav terrain_lib)
ament_target_dependencies(hello_finenav ${dependencies})

add_executable(goal_pose_bridge_node src/goal_pose_bridge_node.cpp)
ament_target_dependencies(goal_pose_bridge_node ${dependencies})

install(TARGETS terrain_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS hello_finenav
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS goal_pose_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
        src/adapter.py
        src/nav_manager.py
        DESTINATION share/${PROJECT_NAME}/src
)

install(FILES
        src/map.pgm
        src/map.yaml
        DESTINATION share/${PROJECT_NAME}/src
)
```

---

### 📄 finenav_demo/stvl_with_mppi/config/finenav_params.yaml

```yaml
# FineNav Engine Parameters
#
# 本文件统一管理 FineNavEngine 中全局单例的 generate_parameter_library 参数：
#   - FineNavLocalizer  （私有子节点 /hello_finenav/finenav_localizer）
#   - Navigator         （私有子节点 /hello_finenav/finenav_navigator）
#
# 与 mppi_params.yaml 一并在 launch 文件中传入同一 Node 即可生效。
# ──────────────────────────────────────────────────────────────────────────────

# ══════════════════════════════════════════════════════════════════════════════
# FineNavLocalizer
# 完整节点路径: /hello_finenav/finenav_localizer
# ══════════════════════════════════════════════════════════════════════════════
hello_finenav/finenav_localizer:
  ros__parameters:
    # ESKF 更新循环频率 [Hz]
    update_rate_hz: 100.0
    # 最大可容忍的观测延迟 [s]
    max_delay_sec: 0.05
    # 位姿观测复用次数（R 矩阵按此倍数缩放）
    pose_smoothing_steps: 1
    # 速度观测复用次数
    twist_smoothing_steps: 1
    # dt 上限 [s]
    max_dt_sec: 0.5
    # 每类观测队列容量
    max_obs_queue_size: 64
    model:
      # 线速度过程噪声标准差 [m/s/√s]
      vel_noise_std: 0.10
      # 角速度过程噪声标准差 [rad/s/√s]
      omega_noise_std: 0.10


# ══════════════════════════════════════════════════════════════════════════════
# Navigator
# 完整节点路径: /hello_finenav/finenav_navigator
# ══════════════════════════════════════════════════════════════════════════════
hello_finenav/finenav_navigator:
  ros__parameters:
    # Action Server 名称（客户端向此名称发送 NavigateToPose 目标）
    action_name: "navigate_to_pose"
    # 行为树 tick 频率 [Hz]
    tick_frequency: 50
    # BT 插件库列表（格式: "package_name/subfolder"）
    # finenav_navigator 的条件插件安装在 lib/plugins/
    # TODO: ：generate_parameter_library 的 unique<>: null 验证器在收到显式传入的空数组 [] 时会触发 InvalidParameterValueException
    plugins:
      - "finenav_navigator/lib/plugins"
    # 行为树 XML 文件目录（格式: "package_name/subfolder"）
    # navigate_to_pose.xml 安装在 share/finenav_navigator/behavior_trees/
    behavior_trees:
      - "finenav_navigator/behavior_trees"

# ══════════════════════════════════════════════════════════════════════════════
# HelloFinenav Main Node
# 包含地形分析、STVL等模块参数
# ══════════════════════════════════════════════════════════════════════════════
hello_finenav:
  ros__parameters:
    terrain_analysis:
      cost_scaling_factor: 1.0
      robot_height: 1.0
      max_gradient: 0.8
    stvl_manager:
      decay_time_sec: 0.1
      decay_time_fov_sec: 0.01
      min_range: 0.0
      max_range: 50.0
      min_angle: 0.0
      max_angle: 1.047 # M_PI / 3.0
```

---

### 📄 finenav_demo/stvl_with_mppi/config/mppi_params.yaml

```yaml

# MPPI Controller Parameters for FineNav-Engine
#
# Parameter path:  /hello_finenav/mppi_layer  (pnode created by ControlLayer)
#                  under prefix  finenav_mppi_controller.*
#
# ──────────────────────────────────────────────────────────────────────────────
# 注意：与 nav2 原版的差异
# ──────────────────────────────────────────────────────────────────────────────
# 1. critics 列表【无法动态配置】
#    finenav_mppi_controller 中所有 Critic（ConstraintCritic / CostCritic /
#    GoalCritic / GoalAngleCritic / ObstaclesCritic / PathAlignCritic /
#    PathAlignLegacyCritic / PathAngleCritic / PathFollowCritic /
#    PreferForwardCritic / TwirlingCritic / VelocityDeadbandCritic）均被
#    硬编码注册（controller.hpp 75-86 行）。
#    若要新增或移除 Critic，需修改源码重新编译。
#    在本配置文件中通过各 Critic 的 enabled 字段来启用/禁用。
#
# 2. 以下 nav2 参数在 finenav_mppi_controller 中【尚未实现】，设置无效：
#      ax_max / ax_min / ay_max / ay_min / az_max  （加速度约束）
#      prune_distance                              （轨迹裁剪距离）
#      reset_period                                （重置周期）
#      regenerate_noises                           （噪声重新生成）
#      visualize / TrajectoryVisualizer            （轨迹可视化开关）
#      PathAngleCritic.mode                        （路径角度评估模式）
#
# 3. 不再支持 RotationShimController 外壳，直接使用 MPPIController。
# ──────────────────────────────────────────────────────────────────────────────

/hello_finenav/mppi_layer:
  ros__parameters:
    finenav_mppi_controller:

      # ── 轨迹优化器基础参数 ─────────────────────────────────────────────────
      time_steps: 30          # 每条候选轨迹的时间步数
      model_dt: 0.05          # 单步时长 (s)，确保 controller_period == model_dt
      batch_size: 3000       # 每次迭代的采样轨迹数
      iteration_count: 1      # 每个控制周期内的优化迭代次数（实时性考虑设为 1）
      temperature: 0.3        # 0→ 只取最优轨迹；很大 → 近均匀平均
      gamma: 0.015            # 控制输入能量正则化系数
      motion_model: "DiffDrive"  # 运动模型: DiffDrive | Omni | Ackermann

      # ── 速度约束 ──────────────────────────────────────────────────────────
      vx_max: 0.5             # 最大前向速度 (m/s)
      vx_min: -0.35           # 最小前向速度（负值允许倒退）
      vy_max: 0.0             # 最大横向速度（差速底盘设为 0）
      wz_max: 2.0            # 最大角速度 (rad/s)

      # ── 噪声采样标准差 ────────────────────────────────────────────────────
      vx_std: 0.5             # vx 采样标准差
      vy_std: 0.2             # vy 采样标准差
      wz_std: 0.4             # wz 采样标准差

      # ══════════════════════════════════════════════════════════════════════
      # Critic 参数
      # 所有 Critic 均已注册，通过 enabled 字段控制是否参与评分
      # ══════════════════════════════════════════════════════════════════════

      # ConstraintCritic：惩罚超出动力学约束的轨迹
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0

      # GoalCritic：距目标足够近时，引导机器人直接趋向目标点
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5   # 距目标 ≤ 此距离时激活 (m)

      # GoalAngleCritic：距目标足够近时，惩罚朝向与目标姿态偏差
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5

      # PreferForwardCritic：鼓励前进，抑制倒退
      PreferForwardCritic:
        enabled: false            # 差速底盘若无需倒退可开启
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5

      # CostCritic：直接使用 costmap 代价进行避障惩罚（推荐启用）
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 100.0      # costmap 代价 ≥ 253 时施加此惩罚
        consider_footprint: true  # 使用完整 footprint 检查
        collision_cost: 1000000.0 # 碰撞轨迹惩罚
        near_goal_distance: 1.0   # 距目标 ≤ 此距离时关闭 CostCritic (m)
        trajectory_point_step: 2  # 评估时跳过的轨迹点数（性能优化）

      # ObstaclesCritic：基于膨胀层的斥力场惩罚（与 CostCritic 二选一）
      ObstaclesCritic:
        enabled: false
        cost_power: 1
        consider_footprint: true
        collision_cost: 10.0
        collision_margin_distance: 0.05
        critical_weight: 20.0
        near_goal_distance: 0.5
        repulsion_weight: 2.5
        inflation_radius: 0.5     # 膨胀半径 (m)，需与 costmap 膨胀层匹配
        cost_scaling_factor: 4.0  # 膨胀代价缩放系数，需与 costmap 膨胀层匹配

      # PathAlignCritic：鼓励局部轨迹方向与全局路径对齐
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05  # 允许全局路径穿过占用格的最大比例
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20        # 只有轨迹规划足够远时才考虑此 Critic
        use_path_orientations: false    # 是否跟随全局规划的朝向（Hybrid A* 时开启）

      # PathAlignLegacyCritic：旧版路径对齐，通常用 PathAlignCritic 代替
      PathAlignLegacyCritic:
        enabled: false
        cost_power: 1
        cost_weight: 10.0
        max_path_occupancy_ratio: 0.07
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false

      # PathFollowCritic：鼓励贴近全局路径（距离最近化）
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 15
        threshold_to_consider: 0.5

      # PathAngleCritic：惩罚与全局路径前方点夹角过大的轨迹
      # 注意：PathAngleCritic.mode 在 finenav_mppi_controller 中尚未实现
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 15
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0

      # VelocityDeadbandCritic：惩罚死区内的速度（防止低速抖动）
      VelocityDeadbandCritic:
        enabled: false
        cost_power: 1
        cost_weight: 35.0
        deadband_velocities: [0.05, 0.05, 0.2]  # [vx, vy, wz]

      # TwirlingCritic：惩罚全向底盘不必要的自旋（差速底盘可关闭）
      TwirlingCritic:
        enabled: false
        cost_power: 1
        cost_weight: 10.0

```

---

### 📄 finenav_demo/stvl_with_mppi/include/stvl_manager.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef STVL_MANAGER_HPP_
#define STVL_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "grid_map.hpp"
#include "voxel.hpp"

namespace finenav {

struct PositionHash {
    std::size_t operator()(const Position & p) const noexcept {
        std::size_t hx = std::hash<double>{}(p.x());
        std::size_t hy = std::hash<double>{}(p.y());
        std::size_t hz = std::hash<double>{}(p.z());
        auto hash_combine = [](std::size_t seed, std::size_t v) {
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        };
        std::size_t seed = 0;
        seed = hash_combine(seed, hx);
        seed = hash_combine(seed, hy);
        seed = hash_combine(seed, hz);
        return seed;
    }
};

struct PositionEqual {
    bool operator()(const Position & a, const Position & b) const noexcept {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

class StvlManager {
public:
    /// @param node  ROS节点
    explicit StvlManager(rclcpp::Node::SharedPtr node);

    /// @param T_map_body  当前车体在地图坐标系下的位姿，由外部定位模块提供
    void PruneExpiredCells(GridMap<Voxel> & map, const Eigen::Isometry3d & T_map_body);

    /// @param T_map_body  当前车体在地图坐标系下的位姿，用于将点云从 base_link 变换到 map 系
    void InsertPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, GridMap<Voxel> & map,
                          const Eigen::Isometry3d & T_map_body);

private:
    static bool isInFrustum(const Eigen::Isometry3d & T_map_body, const Position & p_world,
                            double min_range, double max_range,
                            double min_angle, double max_angle);

    rclcpp::Node::SharedPtr node_;

    DynamicPosSet dynamic_cells_;
    double decay_time_sec_     = 1000000.0;
    double decay_time_fov_sec_ = 1000000.0;
    // FOV parameters
    double min_range_ = 0.0;
    double max_range_ = 50.0;
    double min_angle_ = 0.0;
    double max_angle_ = M_PI / 3.0;
};

} // namespace finenav

#endif // STVL_MANAGER_HPP_
```

---

### 📄 finenav_demo/stvl_with_mppi/include/terrain_analysis.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "grid_map.hpp"
#include "finenav_core/map/map_server.hpp"
#include <cmath>
#include <span>
#include <algorithm>
#include <cstring>

#include <opencv2/opencv.hpp>
#include "voxel.hpp"

#include "finenav_util/cloud_publish_helper.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>


class TerrainAnalyzer {
public:
    /**
     * @brief 构造函数。map_server 仅用于查询地图初始尺寸，不存储。
     *        passability_helper / costmap_helper 由外部（main）持有，TerrainAnalyzer
     *        仅持有非拥有指针，生命周期由调用方保证。
     */
    explicit TerrainAnalyzer(
        const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper = nullptr,
        std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper = nullptr);

    /**
     * @brief 执行一次完整的地形分析与膨胀，并发布可视化点云。
     *        必须在 MapServer 的 postUpdateHook 中调用，此时调用方已持有写锁，
     *        本函数直接使用传入的 map 引用，不会再次获取锁。
     */
    void update(float robot_pose_z, const finenav::GridMap<Voxel>& map);

    /**
     * @brief 根据世界坐标查询代价值。
     */
    int getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const;

    /**
     * @brief 将距离（栅格数）转换为代价值，使用缓存的分辨率，无需访问地图。
     */
    int ComputeCost(int distance) const;


private:
    void ApplyGroundMeanFilter(Eigen::ArrayXXf& ground_array, int kernel_size);

    void TerrainAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map);
    void MapInflation();
    cv::Mat DistanceTransform();
    void publishMaps(const finenav::GridMap<Voxel>& map);

    rclcpp::Node::SharedPtr node_;  // 仅用于获取时间戳
    std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper_;
    std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper_;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;

    cv::Mat passability_image_;
    cv::Mat costmap_image_;
    double cost_scaling_factor_{1.0};
    double robot_height_{1.0};
    double max_gradient_{0.8};
    double cached_resolution_{0.1};
};
```

---

### 📄 finenav_demo/stvl_with_mppi/include/terrain_mapview.hpp

```cpp
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
        std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper = nullptr);

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;
    bool isCollision(float x, float y, float theta) const override;
    int getCost(const finenav::Position3D& pos) const override;
    float getRadius() const override;
    float costAtPose(float x, float y, float theta) const override;
    std::string getBaseFrameID() const override;

    void publishLocalMap();

private:
    std::function<bool(const finenav::Position3D&)> CollisionRule() const;
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

    std::unique_ptr<finenav::OBBCollisionModel> collision_model_;
};
```

---

### 📄 finenav_demo/stvl_with_mppi/include/voxel.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once  // ← 添加这行

#include <cmath>
#include <rclcpp/rclcpp.hpp>
struct Voxel {
    float z_height = NAN;
    bool is_static = false;
    rclcpp::Time timestamp{};
};


```

---

### 📄 finenav_demo/stvl_with_mppi/launch/navigation_stack.launch.py

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    finenav_demo_pkg_dir = get_package_share_directory('finenav_demo')
    mppi_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'mppi_params.yaml'
    )
    finenav_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'finenav_params.yaml'
    )

    finenav_simulator_pkg_dir = get_package_share_directory('finenav_simulator')
    
    # This includes FAST-LIO, etc.
    slam_and_localization_launch_file = os.path.join(
        finenav_simulator_pkg_dir,
        'launch',
        'complete',
        'slam_and_localization.launch.py'
    )

    start_slam_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_and_localization_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'lio_type': 'static', # As in the original stvl_with_mppi.launch.py
            'publish_map_tf': 'true',
        }.items()
    )

    start_hello_finenav = Node(
        package='finenav_demo',
        executable='hello_finenav',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            mppi_params_file,
            finenav_params_file,
        ]
    )

    start_goal_pose_bridge = Node(
        package='finenav_demo',
        executable='goal_pose_bridge_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Also include the adapter we created
    adapter_script = os.path.join(
        finenav_demo_pkg_dir,
        'src',
        'adapter.py'
    )
    start_adapter = Node(
        executable='python3',
        arguments=[adapter_script],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(start_slam_and_localization)
    ld.add_action(start_hello_finenav)
    ld.add_action(start_goal_pose_bridge)
    ld.add_action(start_adapter)

    return ld
```

---

### 📄 finenav_demo/stvl_with_mppi/launch/stvl_with_mppi.launch.py

```python
#  Copyright (c) 2026. 
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China. 
#  All rights reserved. 
 
import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.actions import Node, SetParameter
 
def generate_launch_description(): 
    finenav_demo_pkg_dir = get_package_share_directory('finenav_demo') 
    mppi_params_file = os.path.join( 
        finenav_demo_pkg_dir, 
        'config', 
        'mppi_params.yaml' 
    ) 
    finenav_params_file = os.path.join( 
        finenav_demo_pkg_dir, 
        'config', 
        'finenav_params.yaml' 
    ) 
 
    finenav_simulator_pkg_dir = get_package_share_directory('finenav_simulator') 
    finenav_simulator_launch_file = os.path.join( 
        finenav_simulator_pkg_dir, 
        'launch', 
        'complete', 
        'finenav_simulator.launch.py' 
    ) 
 
    start_finenav_simulator = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(finenav_simulator_launch_file), 
        launch_arguments={ 
            'use_sim_time': 'True', 
            'lio_type': 'static', 
            'world_name': 'flat_small_scale_maze.world', 
            'publish_map_tf': 'True', 
        }.items() 
    ) 
 
    start_hello_finenav = Node( 
        package='finenav_demo', 
        executable='hello_finenav', 
        output='screen', 
        parameters=[ 
            {'use_sim_time': True}, 
            mppi_params_file, 
            finenav_params_file, 
        ] 
    ) 
 
    start_goal_pose_bridge = Node( 
        package='finenav_demo', 
        executable='goal_pose_bridge_node', 
        output='screen', 
        parameters=[ 
            {'use_sim_time': True}, 
        ] 
    ) 

    # Adapter node for HTTP map upload
    adapter_script = os.path.join(
        finenav_demo_pkg_dir,
        'src',
        'adapter.py'
    )
    start_adapter = Node(
        executable='python3',
        arguments=[adapter_script],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
 
    ld = LaunchDescription() 
    
    # Global simulation time parameter for all nodes
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    ld.add_action(start_finenav_simulator) 
    ld.add_action(start_hello_finenav) 
    ld.add_action(start_goal_pose_bridge) 
    ld.add_action(start_adapter)
 
    return ld
```

---

### 📄 finenav_demo/stvl_with_mppi/src/adapter.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
import os
import cv2
import threading
import math
import yaml
from flask import Flask, request, jsonify

# 尝试导入自定义服务，如果不存在则定义备用类
try:
    from finenav_msgs.srv import SetMap
except ImportError:
    class SetMap:
        class Request:
            def __init__(self): self.map = OccupancyGrid()
        class Response:
            def __init__(self): self.success = False; self.message = ""

class CloudToMapAdapter(Node):
    def __init__(self):
        super().__init__('cloud_to_map_adapter')

        # 声明参数（ROS 2 会自动根据 Launch 文件覆盖这些值）
        self.declare_parameter('min_z', 0.2)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 1000)
        self.declare_parameter('height', 1000)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 地图初始化 (-1: 未知, 100: 占据, 0: 空闲)
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        self.is_mapping = False
        self.has_disk_map = False
        self.initial_pose_in_odom = None 

        # TF 监听器（自动关联节点时钟）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布者与订阅者
        self.cloud_sub = self.create_subscription(PointCloud2, '/cloud_for_lio', self.cloud_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # 服务
        self.start_srv = self.create_service(Trigger, '/start_mapping', self.start_mapping_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_mapping', self.stop_mapping_callback)
        
        try:
            from finenav_msgs.srv import SetMap as ActualSetMap
            self.set_map_srv = self.create_service(ActualSetMap, '/set_map', self.set_map_callback)
        except ImportError:
            self.get_logger().error("finenav_msgs.srv.SetMap not found.")

        # 定时器：Pose 发布频率高些，Map 发布频率低些
        self.timer = self.create_timer(0.05, self.publish_pose)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        # 加载本地已有地图
        self.load_map()

        # Flask HTTP 服务
        self.flask_app = Flask(__name__)
        self.setup_flask_routes()
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        self.get_logger().info("Cloud to Map Adapter with Correct TimeSync Initialized.")

    def setup_flask_routes(self):
        @self.flask_app.route('/upload_map', methods=['POST'])
        def upload_map():
            if 'pgm' not in request.files or 'yaml' not in request.files:
                return jsonify({"success": False, "message": "Missing files"}), 400
            
            dir_path = os.path.dirname(os.path.realpath(__file__))
            try:
                request.files['pgm'].save(os.path.join(dir_path, "map.pgm"))
                request.files['yaml'].save(os.path.join(dir_path, "map.yaml"))
                self.load_map()
                self.publish_map()
                return jsonify({"success": True, "message": "Map updated via HTTP"})
            except Exception as e:
                return jsonify({"success": False, "message": str(e)}), 500

    def run_flask(self):
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

    def cloud_callback(self, msg):
        if not self.is_mapping:
            return

        try:
            # 关键：查询点云产生那一时刻的位姿
            T_odom_cloud = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # 解析变换
            tx_oc = T_odom_cloud.transform.translation.x
            ty_oc = T_odom_cloud.transform.translation.y
            q_oc = T_odom_cloud.transform.rotation
            yaw_oc = math.atan2(2 * (q_oc.w * q_oc.z + q_oc.x * q_oc.y), 1 - 2 * (q_oc.y * q_oc.y + q_oc.z * q_oc.z))

            if self.initial_pose_in_odom:
                # 转换到起始建图坐标系
                tx_oi = self.initial_pose_in_odom.transform.translation.x
                ty_oi = self.initial_pose_in_odom.transform.translation.y
                q_oi = self.initial_pose_in_odom.transform.rotation
                yaw_oi = math.atan2(2 * (q_oi.w * q_oi.z + q_oi.x * q_oi.y), 1 - 2 * (q_oi.y * q_oi.y + q_oi.z * q_oi.z))
                
                dx, dy = tx_oc - tx_oi, ty_oc - ty_oi
                tx = dx * math.cos(yaw_oi) + dy * math.sin(yaw_oi)
                ty = -dx * math.sin(yaw_oi) + dy * math.cos(yaw_oi)
                tz = T_odom_cloud.transform.translation.z - self.initial_pose_in_odom.transform.translation.z
                yaw = yaw_oc - yaw_oi
            else:
                tx, ty, tz, yaw = tx_oc, ty_oc, T_odom_cloud.transform.translation.z, yaw_oc

            # 处理点云数据
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            sx, sy = int((tx - self.origin_x) / self.resolution), int((ty - self.origin_y) / self.resolution)
            
            free_mask = np.zeros_like(self.grid, dtype=np.uint8)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            for p in points:
                lx, ly, lz = p
                gx = lx * cos_y - ly * sin_y + tx
                gy = lx * sin_y + ly * cos_y + ty
                gz = lz + tz
                ix, iy = int((gx - self.origin_x) / self.resolution), int((gy - self.origin_y) / self.resolution)

                if 0 <= ix < self.width and 0 <= iy < self.height:
                    if self.min_z <= gz <= self.max_z:
                        if 0 <= sx < self.width and 0 <= sy < self.height:
                            cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)
                        self.grid[iy, ix] = 100
                    elif gz < self.min_z:
                        if 0 <= sx < self.width and 0 <= sy < self.height:
                            cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)

            self.grid[(free_mask == 1) & (self.grid == -1)] = 0

        except Exception as e:
            pass

    def publish_pose(self):
        try:
            # 获取最新变换（Time(0) 在仿真模式下指向最新仿真时间）
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg() # 关键：使用节点仿真时钟
            pose.header.frame_id = self.odom_frame
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t.transform.rotation
            self.pose_pub.publish(pose)
        except:
            pass

    def publish_map(self):
        if not self.is_mapping and not self.has_disk_map:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg() # 关键：使用节点仿真时钟
        msg.header.frame_id = self.odom_frame
        msg.info.resolution = self.resolution
        msg.info.width, msg.info.height = self.width, self.height
        
        if self.is_mapping and self.initial_pose_in_odom:
            tx_oi = self.initial_pose_in_odom.transform.translation.x
            ty_oi = self.initial_pose_in_odom.transform.translation.y
            q_oi = self.initial_pose_in_odom.transform.rotation
            yaw_oi = math.atan2(2 * (q_oi.w * q_oi.z + q_oi.x * q_oi.y), 1 - 2 * (q_oi.y * q_oi.y + q_oi.z * q_oi.z))
            msg.info.origin.position.x = tx_oi + self.origin_x * math.cos(yaw_oi) - self.origin_y * math.sin(yaw_oi)
            msg.info.origin.position.y = ty_oi + self.origin_x * math.sin(yaw_oi) + self.origin_y * math.cos(yaw_oi)
            msg.info.origin.orientation = q_oi
        else:
            msg.info.origin.position.x, msg.info.origin.position.y = self.origin_x, self.origin_y
            msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def start_mapping_callback(self, request, response):
        self.is_mapping = True
        self.has_disk_map = False
        try:
            self.initial_pose_in_odom = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except:
            self.initial_pose_in_odom = None
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        response.success = True
        response.message = "Mapping started with sync clock."
        return response

    def stop_mapping_callback(self, request, response):
        self.is_mapping = False
        response.success = self.save_map()
        return response

    def set_map_callback(self, request, response):
        m = request.map
        self.resolution, self.width, self.height = m.info.resolution, m.info.width, m.info.height
        self.origin_x, self.origin_y = m.info.origin.position.x, m.info.origin.position.y
        self.grid = np.array(m.data, dtype=np.int8).reshape((self.height, self.width))
        self.save_map()
        self.publish_map()
        response.success = True
        return response

    def load_map(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        pgm_path, yaml_path = os.path.join(dir_path, "map.pgm"), os.path.join(dir_path, "map.yaml")
        if not os.path.exists(pgm_path) or not os.path.exists(yaml_path): return
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.resolution = data['resolution']
                self.origin_x, self.origin_y = data['origin'][0], data['origin'][1]
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                self.height, self.width = img.shape
                img = np.flipud(img)
                self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
                self.grid[img < 10] = 100
                self.grid[img > 245] = 0
                self.has_disk_map = True
                self.get_logger().info("Map loaded from disk.")
        except Exception as e:
            self.get_logger().error(f"Load failed: {e}")

    def save_map(self):
        try:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            pgm_path, yaml_path = os.path.join(dir_path, "map.pgm"), os.path.join(dir_path, "map.yaml")
            pgm_data = np.full((self.height, self.width), 205, dtype=np.uint8)
            pgm_data[self.grid == 100] = 0
            pgm_data[self.grid == 0] = 255
            cv2.imwrite(pgm_path, np.flipud(pgm_data))
            with open(yaml_path, 'w') as f:
                f.write(f"image: map.pgm\nresolution: {self.resolution}\norigin: [{self.origin_x}, {self.origin_y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            return True
        except: return False

def main(args=None):
    # 关键：传入命令行参数以接收 use_sim_time:=true
    rclpy.init(args=args)
    node = CloudToMapAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 📄 finenav_demo/stvl_with_mppi/src/goal_pose_bridge_node.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/navigate_to_pose.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

namespace {

constexpr uint32_t TRAJ_POSE = 1;

geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(0.5 * yaw);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    return q;
}

} // namespace

class GoalPoseLinePathTestNode : public rclcpp::Node {
public:
    using NavigateToPose        = finenav_msgs::action::NavigateToPose;
    using GoalHandleNavigate    = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalPoseLinePathTestNode()
    : rclcpp::Node("goal_pose_bridge_node")
    {
        constexpr const char* kGoalTopic        = "/goal_pose";
        constexpr const char* kPathTopic        = "/hello_finenav/line_path";
        constexpr const char* kNavStateTopic    = "/nav_state";
        constexpr const char* kMapFrame         = "map";
        constexpr const char* kActionServer     = "/hello_finenav/navigate_to_pose";  // Navigator action server
        constexpr const char* kBehaviorTree     = "NavigateToPose";    // BT id in navigate_to_pose.xml
        constexpr const char* kMppiPlanTopic    = "/hello_finenav/mppi_layer_plan";
        constexpr const char* kNavPathTopic     = "/mppi_path";
        constexpr const char* kLinePathNavTopic = "/path";
        constexpr int kNumPoints = 50;

        path_topic_          = kPathTopic;
        map_frame_           = kMapFrame;
        action_server_name_  = kActionServer;
        behavior_tree_       = kBehaviorTree;
        mppi_plan_topic_     = kMppiPlanTopic;
        nav_path_topic_      = kNavPathTopic;
        line_path_nav_topic_ = kLinePathNavTopic;
        num_points_          = kNumPoints;

        // 发布者
        path_pub_          = this->create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);
        nav_path_pub_      = this->create_publisher<nav_msgs::msg::Path>(nav_path_topic_, 10);
        line_path_nav_pub_ = this->create_publisher<nav_msgs::msg::Path>(line_path_nav_topic_, 10);

        // NavigateToPose Action 客户端（指向 Navigator）
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_server_name_);

        // 订阅 nav_state 获取机器人位姿
        nav_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            kNavStateTopic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                latest_pose_ = msg->pose.pose;
                has_pose_ = true;
            });

        // 订阅 /goal_pose
        goal_sub_raw_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            kGoalTopic, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->onGoal(msg);
            });

        // 订阅 MPPI 规划结果并可视化
        mppi_plan_sub_ = this->create_subscription<finenav_msgs::msg::Trajectory>(
            mppi_plan_topic_, 10,
            std::bind(&GoalPoseLinePathTestNode::mppiPlanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Ready. goal_topic=%s  path_topic=%s  nav_action_server=%s  bt=%s",
                    kGoalTopic, path_topic_.c_str(),
                    action_server_name_.c_str(), behavior_tree_.c_str());
    }

private:
    void mppiPlanCallback(const finenav_msgs::msg::Trajectory::SharedPtr msg)
    {
        if (!(msg->valid_fields & TRAJ_POSE) || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "MPPI plan has no pose data, skipping");
            return;
        }
        nav_path_pub_->publish(convertTrajectoryToPath(*msg));
    }

    nav_msgs::msg::Path convertTrajectoryToPath(const finenav_msgs::msg::Trajectory& traj)
    {
        nav_msgs::msg::Path path;
        path.header.stamp    = this->now();
        path.header.frame_id = map_frame_;
        path.poses.reserve(traj.poses.size());
        for (const auto& p : traj.poses) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose   = p;
            path.poses.push_back(ps);
        }
        return path;
    }

    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Pose start_pose;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            if (!has_pose_) {
                RCLCPP_WARN(this->get_logger(),
                            "No nav_state received yet, cannot build reference path.");
                return;
            }
            start_pose = latest_pose_;
        }

        const double sx = start_pose.position.x;
        const double sy = start_pose.position.y;
        const double sz = start_pose.position.z;
        const double gx = msg->pose.position.x;
        const double gy = msg->pose.position.y;
        const double gz = msg->pose.position.z;

        RCLCPP_INFO(this->get_logger(),
                    "Goal received: start=(%.2f, %.2f) -> goal=(%.2f, %.2f)",
                    sx, sy, gx, gy);

        const double dx = gx - sx, dy = gy - sy, dz = gz - sz;
        if (std::sqrt(dx*dx + dy*dy + dz*dz) < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Start and goal coincide, skipping.");
            return;
        }

        // 1. 生成直线参考轨迹并发布到 line_path（ControlLayer prePlan 读取）
        auto ref_traj = generateStraightLineTrajectory(sx, sy, sz, gx, gy, gz);
        path_pub_->publish(ref_traj);
        line_path_nav_pub_->publish(convertTrajectoryToPath(ref_traj));
        RCLCPP_INFO(this->get_logger(),
                    "Published reference trajectory: %zu points", ref_traj.poses.size());

        // 2. 向 Navigator 发送 NavigateToPose action goal
        sendNavigateGoal(*msg);
    }

    finenav_msgs::msg::Trajectory generateStraightLineTrajectory(
        double sx, double sy, double sz,
        double gx, double gy, double gz)
    {
        auto traj = finenav_msgs::msg::Trajectory();
        traj.header.stamp    = this->now();
        traj.header.frame_id = map_frame_;
        traj.valid_fields    = TRAJ_POSE;
        traj.poses.reserve(num_points_);

        const double dx  = gx - sx;
        const double dy  = gy - sy;
        const double dz  = gz - sz;
        const double yaw = std::atan2(dy, dx);
        const auto   q   = quatFromYaw(yaw);

        for (int i = 0; i < num_points_; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(num_points_ - 1);
            geometry_msgs::msg::Pose pose;
            pose.position.x  = sx + t * dx;
            pose.position.y  = sy + t * dy;
            pose.position.z  = sz + t * dz;
            pose.orientation = q;
            traj.poses.push_back(pose);
        }
        return traj;
    }

    void sendNavigateGoal(const geometry_msgs::msg::PoseStamped& goal_pose_stamped)
    {
        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(),
                         "NavigateToPose action server '%s' not available.",
                         action_server_name_.c_str());
            return;
        }

        // 直接发送新目标，SimpleActionServer 的 preempt 机制会处理抢断。
        // 不要先 cancel 旧目标，否则 cancel 与新 goal 同时到达时，
        // terminate_all() 会把 pending 一并 abort，导致新目标被丢弃。
        NavigateToPose::Goal goal_msg;
        goal_msg.pose          = goal_pose_stamped;
        goal_msg.behavior_tree = behavior_tree_;

        RCLCPP_INFO(this->get_logger(),
                    "Sending NavigateToPose goal: (%.2f, %.2f)  bt='%s'",
                    goal_pose_stamped.pose.position.x,
                    goal_pose_stamped.pose.position.y,
                    behavior_tree_.c_str());

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        opts.goal_response_callback =
            [this](const GoalHandleNavigate::SharedPtr& gh) {
                if (!gh) {
                    RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal rejected.");
                    return;
                }
                active_goal_handle_ = gh;
                RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted.");
            };

        opts.result_callback =
            [this](const GoalHandleNavigate::WrappedResult& result) {
                active_goal_handle_ = nullptr;
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(),
                                    "✅ Navigation SUCCEEDED (IsGoalReached triggered).");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(),
                                    "Navigation CANCELED (preempted by new goal or user).");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(this->get_logger(),
                                    "⚠️  Navigation ABORTED (plan failed or BT returned FAILURE).");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Navigation: unknown result code.");
                }
            };

        nav_action_client_->async_send_goal(goal_msg, opts);
    }

    // ---------- publishers ----------
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr line_path_nav_pub_;

    // ---------- subscribers ----------
    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr mppi_plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_raw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;

    // ---------- action ----------
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    GoalHandleNavigate::SharedPtr active_goal_handle_;

    // ---------- robot pose cache ----------
    std::mutex pose_mutex_;
    geometry_msgs::msg::Pose latest_pose_{};
    bool has_pose_{false};

    // ---------- params ----------
    std::string path_topic_;
    std::string map_frame_;
    std::string action_server_name_;
    std::string behavior_tree_;
    std::string mppi_plan_topic_;
    std::string nav_path_topic_;
    std::string line_path_nav_topic_;
    int num_points_{50};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseLinePathTestNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

### 📄 finenav_demo/stvl_with_mppi/src/hello_finenav.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>

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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true); // Use time in /clock
    auto node = rclcpp::Node::make_shared("hello_finenav", options); // TODO: inhirent options?

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

    Region3D map_bounds;
    {
        map_bounds = map_server->getLockedReadView()->getWindowBounds();
    }

    my_control_layer->set_on_pre_plan(
    [map_bounds, map_server, localizer, node, trimmed_path_pub](finenav::PrePlanContext& ctx) -> bool {
        // 1. Advance trajectory start to the waypoint nearest to the robot
        const auto& rp = ctx.robot_state.pose.position;
        ctx.ref_traj | finenav::SeekToNearestPoint({rp.x, rp.y, rp.z});

        // 2. Trim trailing waypoints beyond the maximum tracking distance
        //    (use the shorter dimension of the map as a conservative bound)
        // const auto bounds_size = map_bounds.max() - map_bounds.min();
        // const double max_dist = std::min(bounds_size.x(), bounds_size.y());
        // ctx.ref_traj | finenav::TrimByDistance(max_dist);

        // 3. Trim waypoints that fall outside the current map AABB
        const auto current_bounds = map_server->getLockedReadView()->getWindowBounds();
        ctx.ref_traj | finenav::TrimByAABB(current_bounds);

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
```

---

### 📄 finenav_demo/stvl_with_mppi/src/map.yaml

```yaml
image: map_new.pgm
resolution: 0.05
origin: [-10,-10,0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

---

### 📄 finenav_demo/stvl_with_mppi/src/nav_manager.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import os
import signal

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')
        
        self.declare_parameter('launch_file', '')
        self.launch_file = self.get_parameter('launch_file').value
        
        self.process = None
        
        self.start_srv = self.create_service(Trigger, '/start_navigation', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_navigation', self.stop_callback)
        
        self.get_logger().info("Navigation Manager started.")
        
        # Start navigation by default
        self.start_navigation()

    def start_navigation(self):
        if self.process is not None:
            self.get_logger().warn("Navigation is already running.")
            return False, "Navigation is already running."
        
        if not self.launch_file:
            return False, "No launch file specified."

        try:
            # We use a new process group to make sure we can kill all child processes
            self.process = subprocess.Popen(
                ['ros2', 'launch', self.launch_file],
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Started navigation from {self.launch_file}")
            return True, "Navigation started."
        except Exception as e:
            self.get_logger().error(f"Failed to start navigation: {e}")
            return False, str(e)

    def stop_navigation(self):
        if self.process is None:
            self.get_logger().warn("Navigation is not running.")
            return False, "Navigation is not running."
        
        try:
            # Kill the whole process group
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=5)
            self.process = None
            self.get_logger().info("Navigation stopped.")
            return True, "Navigation stopped."
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Navigation process did not stop gracefully, killing it.")
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process = None
            return True, "Navigation force stopped."
        except Exception as e:
            self.get_logger().error(f"Failed to stop navigation: {e}")
            return False, str(e)

    def start_callback(self, request, response):
        success, message = self.start_navigation()
        response.success = success
        response.message = message
        return response

    def stop_callback(self, request, response):
        success, message = self.stop_navigation()
        response.success = success
        response.message = message
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.process:
            node.stop_navigation()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 📄 finenav_demo/stvl_with_mppi/src/stvl_manager.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "stvl_manager.hpp"

using namespace finenav;

StvlManager::StvlManager(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
    node_->declare_parameter<double>("stvl_manager.decay_time_sec", 1000000.0);
    node_->declare_parameter<double>("stvl_manager.decay_time_fov_sec", 1000000.0);
    node_->declare_parameter<double>("stvl_manager.min_range", 0.0);
    node_->declare_parameter<double>("stvl_manager.max_range", 50.0);
    node_->declare_parameter<double>("stvl_manager.min_angle", 0.0);
    node_->declare_parameter<double>("stvl_manager.max_angle", M_PI / 3.0);

    node_->get_parameter("stvl_manager.decay_time_sec", decay_time_sec_);
    node_->get_parameter("stvl_manager.decay_time_fov_sec", decay_time_fov_sec_);
    node_->get_parameter("stvl_manager.min_range", min_range_);
    node_->get_parameter("stvl_manager.max_range", max_range_);
    node_->get_parameter("stvl_manager.min_angle", min_angle_);
    node_->get_parameter("stvl_manager.max_angle", max_angle_);
}

bool StvlManager::isInFrustum(const Eigen::Isometry3d & T_map_body, const Position & p_world,
                               double min_range, double max_range,
                               double min_angle, double max_angle)
{
    const Eigen::Vector3d origin = T_map_body.translation();
    const Eigen::Vector3d v_world = p_world - origin;

    const double r = v_world.norm();
    if (r <= 0.0 || r < min_range || r > max_range) {
        return false;
    }

    const Eigen::Vector3d v_sensor = T_map_body.linear().transpose() * v_world;
    const double norm = v_sensor.norm();
    if (norm <= 0.0) {
        return false;
    }

    const double cos_theta = std::clamp(v_sensor.z() / norm, -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    return theta >= min_angle && theta <= max_angle;
}

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
        if (cell.timestamp.nanoseconds() == 0) {    // TODO: 按逻辑不应出现，后续排查
            it = dynamic_cells_.erase(it);
            continue;
        }

        const double age_sec = (now - cell.timestamp).seconds();
        if (age_sec > decay_time_sec_) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
            continue;
        }

        const bool in_fov = isInFrustum(T_map_body, pos, min_range_, max_range_, min_angle_, max_angle_);
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
```

---

### 📄 finenav_demo/stvl_with_mppi/src/terrain_analysis.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-27.
//
#include "terrain_analysis.hpp"

TerrainAnalyzer::TerrainAnalyzer(
    const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<finenav_utils::CloudPublishHelper> passability_helper,
    std::shared_ptr<finenav_utils::CloudPublishHelper> costmap_helper)
    : node_(node)
    , passability_helper_(std::move(passability_helper))
    , costmap_helper_(std::move(costmap_helper))
{
    const auto map_ro = map_server->getLockedReadView();
    passability_image_ = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
    costmap_image_     = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
    cached_resolution_ = map_ro->getResolution();

    node_->declare_parameter<double>("terrain_analysis.cost_scaling_factor", 0.5);
    node_->declare_parameter<double>("terrain_analysis.robot_height", 1.0);
    node_->declare_parameter<double>("terrain_analysis.max_gradient", 0.8);
    node_->declare_parameter<std::string>("terrain_analysis.global_map_topic", "/map_terrain");

    node_->get_parameter("terrain_analysis.cost_scaling_factor", cost_scaling_factor_);
    node_->get_parameter("terrain_analysis.robot_height", robot_height_);
    node_->get_parameter("terrain_analysis.max_gradient", max_gradient_);

    std::string global_map_topic;
    node_->get_parameter("terrain_analysis.global_map_topic", global_map_topic);

    global_map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        global_map_topic, rclcpp::QoS(1).transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            if (!global_map_) {
                global_map_ = msg;
            }
        });
}

void TerrainAnalyzer::update(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
    cached_resolution_ = map.getResolution();
    TerrainAnalysis(robot_pose_z, map);
    MapInflation();
    publishMaps(map);
}

int TerrainAnalyzer::getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const {
    if (map.isInside(pos)) {
        finenav::Index index = map.getIndex(pos);
        return costmap_image_.at<uchar>(
            index.y() + map.getSize().y() / 2,
            index.x() + map.getSize().x() / 2);
    }
    return 254;
}

int TerrainAnalyzer::ComputeCost(int distance) const {
    const double factor = std::exp(-1.0 * cost_scaling_factor_ * (distance * cached_resolution_));
    return static_cast<int>(253 * factor);
}

void TerrainAnalyzer::TerrainAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
    int sizeX_min = map.getMinIndex().x();
    int sizeX_max = map.getMaxIndex().x();
    int sizeY_min = map.getMinIndex().y();
    int sizeY_max = map.getMaxIndex().y();
    size_t size_x      = map.getSize().x();
    size_t size_y      = map.getSize().y();
    size_t half_size_x = size_x / 2;
    size_t half_size_y = size_y / 2;

    Eigen::ArrayXXf ground_array_ = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

    for (int x = sizeX_min; x <= sizeX_max; ++x) {
        for (int y = sizeY_min; y <= sizeY_max; ++y) {
            // getVoxelsAlongZ now returns a deep-copied std::vector, safe across
            // rolling-buffer shifts in z.
            const std::vector<Voxel> voxels_along_z = map.getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));

            std::vector<Voxel> valid_z_values;
            for (const auto& voxel : voxels_along_z) {
                if (!std::isnan(voxel.z_height)) {
                    valid_z_values.push_back(voxel);
                }
            }

            float nearest_ground = std::numeric_limits<float>::max();
            if (!valid_z_values.empty()) {
                for (size_t i = 0; i < valid_z_values.size() - 1; ++i) {
                    if(valid_z_values[i + 1].z_height - valid_z_values[i].z_height > robot_height_) {
                        if (fabs(valid_z_values[i].z_height - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                            nearest_ground = valid_z_values[i].z_height;
                        }
                    }
                }
                if (fabs(valid_z_values.back().z_height - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                    nearest_ground = valid_z_values.back().z_height;
                }
            } else {
                nearest_ground = NAN;
            }
            ground_array_(x + half_size_x, y + half_size_y) = nearest_ground;
        }
    }

    ApplyGroundMeanFilter(ground_array_, 3);

    for (size_t x = 0; x < size_x; ++x) {
        for (size_t y = 0; y < size_y; ++y) {
            int dx[4] = {0, 1, 0, -1};
            int dy[4] = {1, 0, -1, 0};

            float terrain_value  = 0;
            float current_ground = ground_array_(x, y);
            if (!std::isnan(current_ground)) {
                for (int i = 0; i < 4; ++i) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];
                    if (nx < 0 || nx > static_cast<int>(size_x) - 1 ||
                        ny < 0 || ny > static_cast<int>(size_y) - 1) continue;
                    float neighbor_ground = ground_array_(nx, ny);
                    if (!std::isnan(neighbor_ground)) {
                        if (fabs(current_ground - neighbor_ground) > max_gradient_) {
                            terrain_value = 254;
                            break;
                        }
                    }
                }
            }
            passability_image_.at<uchar>(y, x) = terrain_value;
        }
    }

    // 融合全局占据地图 (0/100 二值图)
    if (global_map_) {
        const double map_res = global_map_->info.resolution;
        const double origin_x = global_map_->info.origin.position.x;
        const double origin_y = global_map_->info.origin.position.y;
        const int map_w = global_map_->info.width;
        const int map_h = global_map_->info.height;

        for (size_t x = 0; x < size_x; ++x) {
            for (size_t y = 0; y < size_y; ++y) {
                finenav::Index idx(
                    static_cast<int>(x) - static_cast<int>(half_size_x),
                    static_cast<int>(y) - static_cast<int>(half_size_y),
                    0);

                if (map.isInside(idx)) {
                    auto pos = map.getPosition(idx);
                    int gx = std::floor((pos.x() - origin_x) / map_res);
                    int gy = std::floor((pos.y() - origin_y) / map_res);

                    if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h) {
                        int g_idx = gy * map_w + gx;
                        if (global_map_->data[g_idx] == 100) {
                            passability_image_.at<uchar>(y, x) = 254; // 标记为不可通行
                        }
                    }
                }
            }
        }
    }
}

void TerrainAnalyzer::ApplyGroundMeanFilter(Eigen::ArrayXXf& ground_array, int kernel_size) {
    int rows = ground_array.rows();
    int cols = ground_array.cols();
    Eigen::ArrayXXf filtered_array = ground_array;
    int half_k = kernel_size / 2;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (std::isnan(ground_array(r, c))) continue;

            float sum = 0.0f;
            int count = 0;

            for (int kr = -half_k; kr <= half_k; ++kr) {
                for (int kc = -half_k; kc <= half_k; ++kc) {
                    int nr = r + kr;
                    int nc = c + kc;
                    if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
                        float val = ground_array(nr, nc);
                        if (!std::isnan(val)) {
                            sum += val;
                            count++;
                        }
                    }
                }
            }

            if (count > 0) {
                filtered_array(r, c) = sum / count;
            }
        }
    }

    ground_array = filtered_array;
}

void TerrainAnalyzer::MapInflation() {
    cv::Mat distance_transform = DistanceTransform();
    for (int x = 0; x < passability_image_.cols; ++x) {
        for (int y = 0; y < passability_image_.rows; ++y) {
            if (passability_image_.at<uchar>(y, x) == 0) {
                float distance = distance_transform.at<float>(y, x);
                costmap_image_.at<uchar>(y, x) = static_cast<uchar>(ComputeCost(distance));
            } else {
                costmap_image_.at<uchar>(y, x) = 254;
            }
        }
    }
}

cv::Mat TerrainAnalyzer::DistanceTransform() {
    cv::Mat passability_binary = 254 - passability_image_;
    cv::Mat distance_transform;
    cv::distanceTransform(passability_binary, distance_transform, cv::DIST_L2, 3);
    return distance_transform;
}

void TerrainAnalyzer::publishMaps(const finenav::GridMap<Voxel>& map) {
    if (!node_) return;

    const size_t size_x      = map.getSize().x();
    const size_t size_y      = map.getSize().y();
    const size_t half_size_x = size_x / 2;
    const size_t half_size_y = size_y / 2;
    const auto   stamp       = node_->now();

    // 像素 (px, py) → 地图 Index (px - half_size_x, py - half_size_y, 0)
    // 与 TerrainAnalysis 中 ground_array_(x+half_size_x, y+half_size_y) 对应

    // ---- passability_map ----
    // 颜色编码: 0（可通行）→ 绿色; 254（障碍）→ 红色; 其余 → 灰色
    if (passability_helper_) {
        passability_helper_->reserve(size_x * size_y);
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y),
                    0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                const uint8_t val = passability_image_.at<uint8_t>(
                    static_cast<int>(py), static_cast<int>(px));
                std::array<uint8_t, 3> color;
                if (val == 0) {
                    color = {0, 200, 0};    // 可通行 → 绿
                } else if (val >= 254) {
                    color = {200, 0, 0};    // 障碍 → 红
                } else {
                    color = {120, 120, 120}; // 其他 → 灰
                }
                passability_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    0.0f, color);
            }
        }
        passability_helper_->publish(stamp);
    }

    // ---- costmap_cloud ----
    // 颜色编码: cost 0（低代价）→ 绿色; cost 254（高代价/障碍）→ 红色
    if (costmap_helper_) {
        costmap_helper_->reserve(size_x * size_y);
        for (size_t px = 0; px < size_x; ++px) {
            for (size_t py = 0; py < size_y; ++py) {
                finenav::Index idx(
                    static_cast<int>(px) - static_cast<int>(half_size_x),
                    static_cast<int>(py) - static_cast<int>(half_size_y),
                    0);
                if (!map.isInside(idx)) continue;
                const auto pos = map.getPosition(idx);
                const uint8_t val = costmap_image_.at<uint8_t>(
                    static_cast<int>(py), static_cast<int>(px));
                // 绿(低代价) → 红(高代价)
                if(val==254){
                    costmap_helper_->addPoint(
                        static_cast<float>(pos.x()),
                        static_cast<float>(pos.y()),
                        0.0f, {0, 0, 0}); // 障碍 → 红
                }else{
                const std::array<uint8_t, 3> color = {val, static_cast<uint8_t>(255 - val), 0};
                costmap_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    0.0f, color);
                }
            }
        }
        costmap_helper_->publish(stamp);
    }
}
```

---

### 📄 finenav_demo/stvl_with_mppi/src/terrain_mapview.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-27.
//

#include "terrain_mapview.hpp"


SimpleGridMapView::SimpleGridMapView(
    const finenav::GridMap<Voxel>& map,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<TerrainAnalyzer> terrain_analyzer,
    std::shared_ptr<finenav_utils::CloudPublishHelper> local_map_helper)
    : map_(map)
    , node_(node)
    , terrain_analyzer_(terrain_analyzer)
    , local_map_helper_(std::move(local_map_helper))
{
    collision_model_ = std::make_unique<finenav::OBBCollisionModel>(finenav::Vector3D(0.8, 0.7, 0.3));
}

bool SimpleGridMapView::isTrackingUnknown() const {
    return is_tracking_unknown_;
}

bool SimpleGridMapView::considerFootprint() const {
    return consider_footprint_;
}

bool SimpleGridMapView::isCollision(float x, float y, float theta) const {
    int pose_cost = getCost(finenav::Position3D{x, y, 0.0});
    finenav::Pose pose = createPose2D(x, y, theta);
    int possible_collision_cost = ComputeCost(collision_model_->getOuterDiameter() / 2 / map_.getResolution());
    if (pose_cost == 254) {
        return true;
    }
    if (pose_cost >= possible_collision_cost) {
        return collision_model_->checkCollision(pose, CollisionRule());
    }
    return false;
}

int SimpleGridMapView::getCost(const finenav::Position3D& pos) const {
    return terrain_analyzer_->getCost(pos, map_);
}

float SimpleGridMapView::getRadius() const {
    return collision_model_->getOuterDiameter();
}

float SimpleGridMapView::costAtPose(float x, float y, float theta) const {
    finenav::Pose pose = createPose2D(x, y, theta);
    return collision_model_->checkCost(pose,[this](const finenav::Position3D& pos) {
        return this->getCost(pos);
    });
}

std::string SimpleGridMapView::getBaseFrameID() const {
    return "map";
}

void SimpleGridMapView::publishLocalMap() {
    if (!local_map_helper_ || !node_) return;

    const size_t size_x = map_.getSize().x();
    const size_t size_y = map_.getSize().y();
    local_map_helper_->reserve(size_x * size_y);

    for (int x = map_.getMinIndex().x(); x <= map_.getMaxIndex().x(); ++x) {
        for (int y = map_.getMinIndex().y(); y < map_.getMaxIndex().y(); ++y) {
            for (int z = map_.getMinIndex().z(); z < map_.getMaxIndex().z(); ++z) {
                finenav::Position3D pos = map_.getPosition({x, y, z});
                const float z_height = map_.atPosition(pos).z_height;
                if (!std::isfinite(z_height)) continue;
                // 用代价值编码颜色：低代价绿，高代价红
                const int cost = getCost(pos);
                const uint8_t cv = static_cast<uint8_t>(std::clamp(cost, 0, 255));
                const std::array<uint8_t, 3> color = {cv, static_cast<uint8_t>(255 - cv), 0};
                local_map_helper_->addPoint(
                    static_cast<float>(pos.x()),
                    static_cast<float>(pos.y()),
                    z_height,
                    color);
            }
        }
    }
    local_map_helper_->publish(node_->now());
}

std::function<bool(const finenav::Position3D&)> SimpleGridMapView::CollisionRule() const {
    return [this](const finenav::Position3D& pos) {
        int pose_cost = this->getCost(pos);
        switch (static_cast<unsigned char>(pose_cost)) {
            case 254:
                return true;
            case 255:
                return is_tracking_unknown_ ? false : true;
        }
        return false;
    };
}

int SimpleGridMapView::ComputeCost(int distance) const {
    return terrain_analyzer_->ComputeCost(distance);
}

finenav::Pose SimpleGridMapView::createPose2D(double x, double y, double theta) const{
    finenav::Pose pose = finenav::Pose::Identity();
    pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    return pose;
}
```

---

### 📄 finenav_engine/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_engine) # Header-only

find_package(ament_cmake REQUIRED)

set(finenav_dependencies
        finenav_core
        finenav_localizer
        finenav_navigator
        finenav_motion_decorator
        finenav_msgs
        finenav_util
)

foreach(dep IN LISTS finenav_dependencies)
    find_package(${dep} REQUIRED)
endforeach()

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${finenav_dependencies})

ament_package()
```

---

### 📄 finenav_engine/include/finenav_engine/finenav_engine.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/navigator.hpp"

namespace finenav {

class FineNavEngine {
public:

    explicit FineNavEngine(const rclcpp::Node::SharedPtr& node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Initializing...");

        localizer_ = std::make_unique<FineNavLocalizer>(node_);
        RCLCPP_INFO(node_->get_logger(), "Localizer ready.");

        navigator_ = std::make_unique<Navigator>(node_);

        // 注入机器人位姿提供函数，供 BT 条件节点（IsGoalReached、IsStuck 等）读取
        navigator_->set_robot_pose_provider([this]() -> geometry_msgs::msg::Pose {
            const auto& state = localizer_->getState();
            geometry_msgs::msg::Pose p;
            p.position.x = state.p.x();
            p.position.y = state.p.y();
            p.position.z = state.p.z();
            const auto q = state.R.unit_quaternion();
            p.orientation.w = q.w();
            p.orientation.x = q.x();
            p.orientation.y = q.y();
            p.orientation.z = q.z();
            return p;
        });

        // 注入机器人速度提供函数，供 IsGoalReached 等条件节点读取
        navigator_->set_robot_twist_provider([this]() -> geometry_msgs::msg::Twist {
            const auto& state = localizer_->getState();
            geometry_msgs::msg::Twist t;
            // getState() returns v in body frame
            t.linear.x  = state.v.x();
            t.linear.y  = state.v.y();
            t.linear.z  = state.v.z();
            t.angular.x = state.omega.x();
            t.angular.y = state.omega.y();
            t.angular.z = state.omega.z();
            return t;
        });
        RCLCPP_INFO(node_->get_logger(), "Navigator ready.");


        RCLCPP_INFO(node_->get_logger(), "All singletons initialized.");
    }

    // 禁止复制，允许移动
    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&) = default;
    FineNavEngine& operator=(FineNavEngine&&) = default;
    ~FineNavEngine() = default;

    const rclcpp::Node::SharedPtr& getNode() const { return node_; }
    FineNavLocalizer* getLocalizer() { return localizer_.get(); }
    Navigator* getNavigator() { return navigator_.get(); }


    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<MapServer<MapT>> createMapResource(
        const std::string& name,
        double update_rate_hz     = 10.0,
        bool enable_shift_window  = false,
        double shift_rate_hz      = 50.0)
    {
        auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                               [&](const auto& pair) { return pair.first == name; });
        if (it == map_resources_.end()) {
            std::shared_ptr<MapServer<MapT>> new_map;
            if (enable_shift_window) {
                new_map = std::make_shared<MapServer<MapT>>(
                    node_, name, update_rate_hz, localizer_.get(), shift_rate_hz);
            } else {
                new_map = std::make_shared<MapServer<MapT>>(node_, name, update_rate_hz);
            }
            map_resources_.emplace_back(name, new_map);
            return new_map;
        }
        return std::any_cast<std::shared_ptr<MapServer<MapT>>>(it->second);
    }

    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<MapServer<MapT>> getMapResource(const std::string& name) {
        auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                               [&](const auto& pair) { return pair.first == name; });
        if (it != map_resources_.end()) {
            return std::any_cast<std::shared_ptr<MapServer<MapT>>>(it->second);
        }
        return nullptr;
    }


    /**
     * @brief 创建 ControlLayer 并自动注入框架回调：
     *   - robot_state_provider : 从 Localizer 读取最新位姿/速度
     *   - should_stop_predicate: 查询 NavStateManager 决定 TrackingPolicy 是否退出
     *
     * 用户仍可在返回的 layer 上调用 set_on_post_plan() 注册应用级输出逻辑。
     */
    template <typename PlannersT, typename ExecutionPolicy = SingleShotPolicy>
    [[nodiscard("ControlLayer was created but the handle was lost!")]]
    std::shared_ptr<ControlLayer<PlannersT, ExecutionPolicy>> createControlLayer(
        const std::string& name,
        ExecutionPolicy policy = ExecutionPolicy{})
    {
        auto layer = std::make_shared<ControlLayer<PlannersT, ExecutionPolicy>>(
            node_, name, std::move(policy));

        // Auto-inject: robot state from Localizer
        layer->set_robot_state_provider([this]() -> finenav_msgs::msg::RobotState {
            const auto& state = localizer_->getState();
            finenav_msgs::msg::RobotState rs;
            rs.pose.position.x = state.p.x();
            rs.pose.position.y = state.p.y();
            rs.pose.position.z = state.p.z();
            const auto q = state.R.unit_quaternion();
            rs.pose.orientation.w = q.w();
            rs.pose.orientation.x = q.x();
            rs.pose.orientation.y = q.y();
            rs.pose.orientation.z = q.z();
            // getState() returns v in body frame
            rs.twist.linear.x  = state.v.x();
            rs.twist.linear.y  = state.v.y();
            rs.twist.linear.z  = state.v.z();
            rs.twist.angular.x = state.omega.x();
            rs.twist.angular.y = state.omega.y();
            rs.twist.angular.z = state.omega.z();
            return rs;
        });


        held_layers_.push_back(layer);
        return layer;
    }


    struct PipelineIO {
        std::string input_topic;
        std::string output_topic;
    };

    template <typename... LayersT>
    void registerPipeline(const PipelineIO& io, std::shared_ptr<LayersT>... layers) {
        static_assert(sizeof...(layers) >= 1, "Pipeline must have at least one layer!");
        auto layer_tuple = std::make_tuple(layers...);

        std::get<0>(layer_tuple)->setInputTopic(io.input_topic);

        constexpr auto last_idx = sizeof...(layers) - 1;
        std::get<last_idx>(layer_tuple)->setOutputTopic(io.output_topic);

        if constexpr (sizeof...(layers) >= 2) {
            wire_layers(layers...);
        }

        (layers->setupDataPorts(), ...);
    }

private:

    template <typename T>
    void wire_layers(std::shared_ptr<T>) {}

    template <typename FirstT, typename SecondT, typename... RestT>
    void wire_layers(std::shared_ptr<FirstT> first,
                     std::shared_ptr<SecondT> second,
                     std::shared_ptr<RestT>... rest) {
        std::string internal_topic = first->getName() + "_to_" + second->getName();
        RCLCPP_WARN_STREAM(node_->get_logger(),
            "Connecting " << first->getName() << " → " << second->getName()
            << " via topic: " << internal_topic);
        first->setOutputTopic(internal_topic);
        second->setInputTopic(internal_topic);
        wire_layers(second, rest...);
    }

    rclcpp::Node::SharedPtr node_;

    // ── Engine-level singletons ───────────────────────────────────────────────
    std::unique_ptr<FineNavLocalizer>    localizer_;
    std::unique_ptr<Navigator>           navigator_;

    // ── Held resources ────────────────────────────────────────────────────────
    std::vector<std::pair<std::string, std::any>> map_resources_;
    std::vector<std::shared_ptr<void>>            held_layers_;
};

} // namespace finenav
```

---

### 📄 finenav_engine/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_engine</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>finenav_core</depend>
    <depend>finenav_localizer</depend>
    <depend>finenav_navigator</depend>
    <depend>finenav_motion_decorator</depend>
    <depend>finenav_msgs</depend>
    <depend>finenav_util</depend>

    <test_depend>ament_cmake_gtest</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_interface/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_interface)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


include_directories(include)

ament_auto_find_build_dependencies()

generate_parameter_library(
        test_map_params
        src/test_map.yaml
)
generate_parameter_library(
        test_planner_params
        src/test_planner.yaml
)

#ament_auto_add_executable(simple_planner_node src/local_layer_node.cpp)
#target_link_libraries(simple_planner_node test_map_params test_planner_params)

#ament_auto_add_library(finenav_manager src/finenav_manager.cpp)
ament_auto_add_executable(test_interface_node src/test.cpp)
ament_auto_add_executable(grid_map_stvl_node src/grid_map_stvl.cpp)
target_link_libraries(test_interface_node test_map_params test_planner_params)
target_link_libraries(grid_map_stvl_node test_map_params test_planner_params tf2::tf2 tf2_ros::tf2_ros tf2_eigen::tf2_eigen)


ament_auto_package()
```

---

### 📄 finenav_interface/include/frustum3d.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <functional>
#include <utility>
#include <algorithm>

#include <Eigen/Core>

namespace finenav::geometry {

struct FrustumSensorPose {
    // Homogeneous transform from sensor frame to world frame: p_world = T_ws * p_sensor_h
    // Layout: [ R_ws(3x3)  t_ws(3x1) ]
    //         [   0 0 0       1     ]
    Eigen::Matrix4d T_ws{Eigen::Matrix4d::Identity()};
};

struct FrustumRange {
    double min_range{0.0};
    double max_range{0.0};
};

struct FrustumAngles {
    // Angle between ray (sensor->point) and sensor Z axis (cone axis) must be in [min_angle, max_angle].
    // Typical cone: min_angle = 0, max_angle = half_aperture.
    double min_angle{0.0};
    double max_angle{0.0};
};

struct FrustumParams3D {
    FrustumSensorPose pose{};
    FrustumRange range{};
    FrustumAngles fov{};
};

/**
 * @brief Simple 3D frustum / cone helper (header-only, map-agnostic).
 *
 * Coordinate convention:
 *  - All points are expressed in a common world/map frame.
 *  - FrustumSensorPose::origin is the sensor position in this frame.
 *  - yaw: rotation around +Z (rad), 0 along +X, positive towards +Y.
 *  - pitch: elevation angle (rad), 0 in XY plane, positive towards +Z.
 */
class Frustum3D {
  public:
    Frustum3D() = default;

    explicit Frustum3D(const FrustumParams3D & params)
        : min_range_(params.range.min_range),
          max_range_(params.range.max_range),
          min_angle_(params.fov.min_angle),
          max_angle_(params.fov.max_angle) {
        setPose(params.pose.T_ws);
    }

    void setParams(const FrustumParams3D & params) {
        min_range_ = params.range.min_range;
        max_range_ = params.range.max_range;
        min_angle_ = params.fov.min_angle;
        max_angle_ = params.fov.max_angle;
        setPose(params.pose.T_ws);
    }

    const Eigen::Vector3d & origin() const { return origin_; }

    /**
     * @brief Check if a 3D point in world frame lies inside the frustum.
     */
    bool contains(const Eigen::Vector3d & p_world) const {
        Eigen::Vector3d v = p_world - origin_;
        const double r2 = v.squaredNorm();
        if (r2 <= 0.0) {
            return false; // at origin or invalid
        }
        const double r = std::sqrt(r2);
        if (r < min_range_ || r > max_range_) {
            return false;
        }

        // Transform to sensor local frame: x' forward, y' left, z' up
        Eigen::Vector3d v_s = R_ws_.transpose() * v;

        // Cone axis is +Z' of sensor frame. Compute angle between v_s and +Z'.
        const double v_norm = v_s.norm();
        if (v_norm <= 0.0) {
            return false;
        }

        const double cos_theta = v_s.z() / v_norm;  // dot(v_s, z_axis) / |v_s|
        const double clamped_cos = std::clamp(cos_theta, -1.0, 1.0);
        const double theta = std::acos(clamped_cos);

        if (theta < min_angle_ || theta > max_angle_) {
            return false;
        }

        return true;
    }

    bool contains(double x, double y, double z) const {
        return contains(Eigen::Vector3d{x, y, z});
    }

  private:
    void setPose(const Eigen::Matrix4d & T_ws) {
        // Extract rotation and translation from homogeneous transform
        R_ws_  = T_ws.topLeftCorner<3,3>();
        origin_ = T_ws.topRightCorner<3,1>();
    }

    Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
    double min_range_{0.0};
    double max_range_{0.0};
    double min_angle_{0.0};
    double max_angle_{0.0};
    Eigen::Matrix3d R_ws_{Eigen::Matrix3d::Identity()};
};



}  // namespace finenav::geometry
```

---

### 📄 finenav_interface/include/simpleTestMap.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_util/algo_configurator.hpp"
#include "finenav_interface/test_map_params.hpp"

namespace finenav {



class SimpleTestMap {
public:
    using DataType = int;
    using ConfigType = test_map::Params;

    SimpleTestMap() = default;

    void configure(const ConfigType &params) {
        // 假设有 resolution 和 name 参数
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}

    }

    void debug_print() const {
        RCLCPP_INFO(rclcpp::get_logger("SimpleTestMap"), "SimpleTestMap debug_print called");
    }

    bool isInside(const Position3D& pos) const {
        return true;
    }

    Region3D getWindowBounds() const {
        return Eigen::AlignedBox3d();
    }

    Position3D getWindowCenter() const {
        return Eigen::Vector3d();
    }

    void shiftWindowTo(const Position3D& pos) {

    }

};
}

FINENAV_REGISTER_ALGO_CONFIG(finenav::SimpleTestMap, test_map)
```

---

### 📄 finenav_interface/include/simpleTestPlanner.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/concepts.hpp" // TODO: 为什么这个文件也必不可少
#include "finenav_util/algo_configurator.hpp"
#include "finenav_interface/test_planner_params.hpp"

namespace finenav {
class SimpleTestPlanner {
public:
    SimpleTestPlanner() = default;

    using ConfigType = test_planner::Params;
    static constexpr IOProfile InputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr IOProfile OutputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };

    class IMapView {
    public:
        virtual ~IMapView() = default;
        virtual double getCost(const int x) const = 0;
    };

    using Context = PlanningContext<SimpleTestPlanner>;
    using Result  = std::optional<OutputDataT<SimpleTestPlanner>>;

    void configure(const ConfigType &params) {
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}
    }

    void reset() {
        // SimpleTestPlanner 无跨 Episode 状态，空实现即可
    }

    Result plan(const Context& ctx, const IMapView& map_view) {
        // 当前机器人状态（位姿 + 速度），由框架通过 RobotStateProvider 注入
        const auto& robot_state = ctx.robot_state;
        const auto& start_pose  = robot_state.pose;   // geometry_msgs::msg::Pose
        const auto& start_twist = robot_state.twist;  // geometry_msgs::msg::Twist

        auto goal_pose = ctx.goal_pose;
        auto ref_traj  = ctx.ref_traj;

        // 执行规划算法，e.g. 生成一条直线路径

        // 装填输出轨迹
        return std::nullopt;
    }
};
}

FINENAV_REGISTER_ALGO_CONFIG(finenav::SimpleTestPlanner, test_planner)
```

---

### 📄 finenav_interface/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>finenav_interface</name>
  <version>0.1.0</version>
  <description>Rolling Grid Map used in FineNav</description>
  <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
  <license>MPL 2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <depend>finenav_control_layers</depend>
  <depend>finenav_map_grid_map</depend>
  <depend>finenav_util</depend>
  <depend>generate_parameter_library</depend>
  <depend>finenav_engine</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

### 📄 finenav_interface/src/grid_map_stvl.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-2.
//
#include "grid_map.hpp"
#include "simpleTestPlanner.hpp"
#include "finenav_engine/finenav_engine.hpp"
#include "finenav_core/map/map_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "frustum3d.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

using namespace finenav;

struct Pointcloud {
    float x;
    float y;
    float z;
};

// 体素中存储：z 轴高度、是否静态、最近一次更新时间戳（ROS2 时间）
struct Voxel {
    float        z_height  = NAN;
    bool         is_static = false;
    rclcpp::Time timestamp{};   // 使用 node->get_clock()->now() 赋值 //TODO:改成位数更小的数据结构
};
// ============================ 动态栅格管理相关 ============================

// 使用 Position 作为 key 的哈希器和相等判断，只在本 demo 中使用
struct PositionHash
{
    std::size_t operator()(const Position & p) const noexcept
    {
        // Position 是 Eigen::Vector3d，直接对三维坐标做 hash
        std::size_t hx = std::hash<double>{}(p.x());
        std::size_t hy = std::hash<double>{}(p.y());
        std::size_t hz = std::hash<double>{}(p.z());

        auto hash_combine = [](std::size_t seed, std::size_t v) {
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        };

        std::size_t seed = 0;
        seed = hash_combine(seed, hx);
        seed = hash_combine(seed, hy);
        seed = hash_combine(seed, hz);
        return seed;
    }
};

struct PositionEqual
{
    bool operator()(const Position & a, const Position & b) const noexcept
    {
        // 由于 Position 将通过 GridMap::getPosition(Index) 得到，理论上是栅格中心的精确坐标，
        // 这里直接用完全相等判断。如果将来引入插值，可在此处添加容差。
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

// 全局（本文件作用域）动态栅格集合与超时时间，仅用于 demo
static DynamicPosSet g_dynamic_cells;          // 存储动态栅格的中心 Position
// 基础超时阈值（视野外或普通情况下）：单位：秒
static double        g_decay_time_sec = 10.0;
static double        g_decay_time_fov_sec = 1.0;           //TODO：：只是简单的调整生命时间，是否有需要是加速衰减速度

static const rclcpp::Logger LOGGER = rclcpp::get_logger("finenav_demo");

using GlobalPlanners = PlannerSet<SimpleTestPlanner>;
using LocalLayers = PlannerSet<SimpleTestPlanner>;
using ReactivePlanners = PlannerSet<SimpleTestPlanner>;
using std::chrono_literals::operator"" ms;

bool numisfinite(float x) {
    if(x==std::numeric_limits<float>::infinity() || x==-std::numeric_limits<float>::infinity() || std::isnan(x)) {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("finenav_demo");

    auto finenav_ptr = std::make_shared<FineNavEngine>(node);

    auto global_layer = finenav_ptr->createControlLayer<GlobalPlanners>("global_layer");
    auto local_layer = finenav_ptr->createControlLayer<LocalLayers>("local_layer");
    auto reactive_layer = finenav_ptr->createControlLayer<ReactivePlanners>("reactive_layer");

    finenav_ptr->registerPipeline(FineNavEngine::PipelineIO{}, global_layer, local_layer, reactive_layer);
    auto map_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);
    auto frustum_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/frustum_voxels", 10);
    // ==============================================================================
    // 测试 MapServer 的完整功能 (Hook机制 + 数据源Push模式)
    // ==============================================================================

    // 1. 初始化一个地图服务器，指定更新频率
    MapServer<GridMap<Voxel>> map_server(node, "test_map_server", 10.0);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(node->get_node_base_interface(),
                                                  node->get_node_timers_interface()));

    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 2. 注册预处理 Hook (在应用融合数据前执行)
    map_server.registerPreUpdateHook([node, tf_buffer, frustum_cloud_pub](GridMap<Voxel>& map) {
        using finenav::geometry::Frustum3D;
        using finenav::geometry::FrustumParams3D;
        using finenav::geometry::FrustumSensorPose;
        using finenav::geometry::FrustumRange;
        using finenav::geometry::FrustumAngles;

        rclcpp::Time now = node->get_clock()->now();

        // 1. 获取 base_lidar 在 map 下的变换
        Eigen::Matrix4d T_ws = Eigen::Matrix4d::Identity();
        try {
            geometry_msgs::msg::TransformStamped tf_stamped =
                tf_buffer->lookupTransform("map", "base_lidar", tf2::TimePointZero);
            Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped.transform);
            T_ws = iso.matrix();
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
            // 保持 T_ws = Identity
        }

        FrustumSensorPose sensor_pose;
        sensor_pose.T_ws = T_ws;

        // 视锥参数
        FrustumRange  range;
        range.min_range = 0.0;
        range.max_range = 50.0;

        FrustumAngles fov;
        fov.min_angle   = 0.0;
        fov.max_angle = M_PI / 3.0;

        FrustumParams3D params{sensor_pose, range, fov};
        Frustum3D frustum(params);

        // 遍历所有当前记录为“动态”的栅格中心 Position，根据时间戳 + 是否在视锥内判断是否需要清除
        for (auto it = g_dynamic_cells.begin(); it != g_dynamic_cells.end(); ) {
            const Position & pos = *it;

            // 如果位置已不在地图有效范围内，直接移除
            if (!map.isInside(pos)) {
                it = g_dynamic_cells.erase(it);
                continue;
            }

            Voxel & cell = map.atPosition(pos);

            // 使用 ROS2 时间计算持续时间
            rclcpp::Duration age = now - cell.timestamp;
            const double age_sec = age.seconds();

            // 首先：若年龄已经超过基础阈值，直接删除，无需再做视锥判断
            if (age_sec > g_decay_time_sec) {
                cell = Voxel{NAN, false, rclcpp::Time{}};
                it   = g_dynamic_cells.erase(it);
                continue;
            }

            // 基于视锥判断，若在视锥内，则使用更短的寿命（加速清除）
            bool in_fov = frustum.contains(pos.x(), pos.y(), pos.z());
            double active_decay = in_fov ? g_decay_time_fov_sec : g_decay_time_sec;

            if (age_sec > active_decay) {
                cell = Voxel{NAN, false, rclcpp::Time{}};
                it   = g_dynamic_cells.erase(it);
            } else {
                ++it;
            }
        }

    });

    // 3. 注册后处理 Hook (在应用融合数据后执行)
    map_server.registerPostUpdateHook(
        [node, map_cloud_pub](const GridMap<Voxel>& map)
        {
            auto now = node->now();
            std::vector<Index> valid_indices;
            map.selectCellsByCondition(valid_indices, [](const Voxel& v) {
                return std::isfinite(v.z_height);
            });
            // std::cout<<"valid_indices size: "<<valid_indices.size()<<std::endl;
            if (valid_indices.empty()) return;

            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header.stamp = now;
            cloud.header.frame_id = "map";
            cloud.height = 1;
            cloud.width = valid_indices.size();

            sensor_msgs::PointCloud2Modifier modifier(cloud);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(valid_indices.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

            for (const auto& idx : valid_indices) {
                const auto pos = map.getPosition(idx);
                const auto& cell = map.at(idx);

                *iter_x = pos.x();
                *iter_y = pos.y();
                *iter_z = cell.z_height;

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            map_cloud_pub->publish(cloud);
            RCLCPP_INFO(LOGGER, "<< [Hook] PostUpdateHook triggered.");

        }
    );

    // 4. 初始化策略并注册数据注射器
    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;
    auto laser_injector = map_server.addUpdateSource<Pointcloud>(
        "front_laser",
        policy,
        [node](const Pointcloud& msg, const rclcpp::Time& stamp, GridMap<Voxel>& map) {
            rclcpp::Time now = node->get_clock()->now();
            // 演示：假设激光击中地图中心 (0,0) 处的某个体素，只更新一次
            // 实际工程中应该根据传感器模型计算出命中位置
            Position hit_pos = Position(msg.x , msg.y, msg.z); // TODO: 根据 msg 计算实际命中位置

            if (!map.isInside(hit_pos)) {
                return;
            }

            Index  idx  = map.getIndex(hit_pos);
            Voxel & cell = map.at(idx);

            if (!cell.is_static) {
                cell.z_height  = msg.z;   // 这里用 range 代替 z，仅作示意
                cell.timestamp = now;         // 记录 ROS2 时间
                // 使用 Position 作为 key 记录为动态栅格（哈希表自动去重）
                Position center_pos = map.getPosition(idx);
                g_dynamic_cells.insert(center_pos);
            }
        }
    );

    // 新增：地图点云可视化发布器（手动触发发布）


    // 订阅点云并用 TF 过滤，保证 TF 可用时才处理点云
    auto cloud_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node, "/cloud_for_lio");
    auto tf_filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *cloud_sub, *tf_buffer, "map", 10,
        node->get_node_logging_interface(), node->get_node_clock_interface(), 10ms);

    tf_filter->registerCallback(
        [node, laser_injector, tf_buffer](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
            geometry_msgs::msg::TransformStamped tf_stamped;
            try {
                tf_stamped = tf_buffer->lookupTransform("map", msg->header.frame_id, msg->header.stamp);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
                return;
            }
            Eigen::Isometry3d T_map_lidar = tf2::transformToEigen(tf_stamped.transform);

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            rclcpp::Time now = node->now();

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if(numisfinite(*iter_x) && numisfinite(*iter_y) && numisfinite(*iter_z)) {
                    Eigen::Vector3d p_lidar(*iter_x, *iter_y, *iter_z);
                    Eigen::Vector3d p_map = T_map_lidar * p_lidar;
                    Pointcloud p;
                    p.x = static_cast<float>(p_map.x());
                    p.y = static_cast<float>(p_map.y());
                    p.z = static_cast<float>(p_map.z());
                    laser_injector(std::move(p), now);
                }
            }
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

```

---

### 📄 finenav_interface/src/test.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine/finenav_engine.hpp"
#include "finenav_core/map/map_server.hpp"
#include "simpleTestMap.hpp"
#include "simpleTestPlanner.hpp"

using namespace finenav;

struct DummyLaserScan {
    int seq;
    float range;
};

static const rclcpp::Logger LOGGER = rclcpp::get_logger("finenav_demo");

using GlobalPlanners = PlannerSet<SimpleTestPlanner>;
using LocalLayers = PlannerSet<SimpleTestPlanner>;
using ReactivePlanners = PlannerSet<SimpleTestPlanner>;

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("finenav_demo");

    auto finenav_ptr = std::make_shared<FineNavEngine>(node);

    auto global_layer = finenav_ptr->createControlLayer<GlobalPlanners>("global_layer");
    auto local_layer = finenav_ptr->createControlLayer<LocalLayers>("local_layer");
    auto reactive_layer = finenav_ptr->createControlLayer<ReactivePlanners>("reactive_layer");

    finenav_ptr->registerPipeline(FineNavEngine::PipelineIO{}, global_layer, local_layer, reactive_layer);

    // ==============================================================================
    // 测试 FineNavLocalizer
    // ==============================================================================
    finenav_ptr->getLocalizer(); // 【注意：目前存在小bug，必须将这个api在createMapResource前调用一次】


    // ==============================================================================
    // 测试 MapServer 的完整功能 (Hook机制 + 数据源Push模式)
    // ==============================================================================

    // 1. 初始化一个地图服务器，指定更新频率 1.0Hz
    auto map_server_ptr = finenav_ptr->createMapResource<SimpleTestMap>(
        "test_map_server",
        1.0,   // update_rate_hz
        true,  // enable_shift_window
        50.0   // shift_rate_hz
    );

    // 2. 注册预处理 Hook (在应用融合数据前执行)
    map_server_ptr->registerPreUpdateHook([](SimpleTestMap& map) {
        RCLCPP_INFO(LOGGER, ">> [Hook] PreUpdateHook triggered. Preparing map...");
    });

    // 3. 注册后处理 Hook (在应用融合数据后执行)
    map_server_ptr->registerPostUpdateHook([](const SimpleTestMap& map) {
        RCLCPP_INFO(LOGGER, "<< [Hook] PostUpdateHook triggered. Map updating complete.");
    });

    // 4. 初始化策略并注册数据注射器
    MapServer<SimpleTestMap>::SourceBufferPolicy policy;
    policy.max_buffer_size = 10;
    policy.observation_keep_time_sec = 2.0;

    auto laser_injector = map_server_ptr->addUpdateSource<DummyLaserScan>(
        "front_laser",
        policy,
        [](const DummyLaserScan& msg, const rclcpp::Time& /*stamp*/, SimpleTestMap& map) {
            RCLCPP_INFO(LOGGER, "   [Fuser] Fusing DummyLaserScan data: Seq = %d, Range = %.2f", msg.seq, msg.range);
            (void)map; // 防止 unused parameter 警告
        }
    );

    // 5. 模拟一个高频的传感器硬件或话题回调，产生数据并持续推入注射器
    std::shared_ptr<int> mock_seq_count = std::make_shared<int>(0);
    auto mock_sensor_timer = node->create_wall_timer(std::chrono::milliseconds(200), [node, laser_injector, mock_seq_count]() {
        DummyLaserScan mock_scan_data{};
        mock_scan_data.seq = (*mock_seq_count)++;
        mock_scan_data.range = 5.0f;

        // 我们只需把传感器数据无脑塞进去，MapServer内部会自动保护并等待周期来临
        // 强制采用 std::move 提供右值，明确所有权移交，注入后数据作废
        laser_injector(std::move(mock_scan_data), node->now());
    });

    // 6. 添加一个定时器，模拟用户主动去访问 Map
    auto mock_user_timer = node->create_wall_timer(std::chrono::milliseconds(500), [&map_server_ptr]() {
        // 用户获取带有读锁的视图，极度丝滑
        auto map_view = map_server_ptr->getLockedReadView();
        // 因为操作符重载，等价于操作 const SimpleTestMap 对象本身的方法，自动线程安全
        RCLCPP_INFO(LOGGER, "   [User] Map accessed safely. LockedReadView protects the thread.");
    });

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
```

---

### 📄 finenav_interface/src/test_map.yaml

```yaml
test_map:
  background:
    r:
      type: int
      default_value: 0
      description: "Red color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    g:
      type: int
      default_value: 0
      description: "Green color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    b:
      type: int
      default_value: 0
      description: "Blue color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
```

---

### 📄 finenav_interface/src/test_planner.yaml

```yaml
test_planner:
  background:
    r:
      type: int
      default_value: 0
      description: "Red color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    g:
      type: int
      default_value: 0
      description: "Green color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    b:
      type: int
      default_value: 0
      description: "Blue color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
```

---

### 📄 finenav_localizer/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_localizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)

# Generate parameter library for FineNavLocalizer
generate_parameter_library(
        localizer_params
        src/finenav_localizer.yaml
)

# FineNav Localizer (ESKF-based localization module)
add_library(${PROJECT_NAME} SHARED
        src/finenav_localizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        tf2_ros
        tf2_eigen
        geometry_msgs
        finenav_util
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        Sophus::Sophus
        localizer_params
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
    add_subdirectory(eval)
endif ()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
# Install the generated parameter header so downstream packages can find it
install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/include/finenav_localizer/localizer_params.hpp
        DESTINATION include/finenav_localizer
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        rclcpp
        tf2_ros
        tf2_eigen
        geometry_msgs
        finenav_util
        Eigen3
        Sophus
        generate_parameter_library
)
ament_package()
```

---

### 📄 finenav_localizer/eval/CMakeLists.txt

```
add_executable(${PROJECT_NAME}_eval
        eval_scenario.cpp
)
target_include_directories(${PROJECT_NAME}_eval PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJECT_NAME}_eval
        finenav_localizer
)
```

---

### 📄 finenav_localizer/eval/eval_config.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  FineNavLocalizer — Scenario Test Configuration                         ║
// ║                                                                          ║
// ║  Edit ONLY this file to change test behaviour between runs.             ║
// ║  Recompile after any change.                                            ║
// ╚══════════════════════════════════════════════════════════════════════════╝

namespace cfg {

// ── 1. Scenario selection ─────────────────────────────────────────────────────
//   1 = Uniform circular motion, all ideal
//   2 = Uniform circular, Session-B stress events
//   3 = High-dynamic Lissajous figure-8, ideal
constexpr int kScenario = 2;

// ── 2. Observation mode ───────────────────────────────────────────────────────
//   At least one of kUsePose / kUseTwist must be true.
//   kUsePose  : feed PoseWithCovarianceStamped  (position + orientation)
//   kUseTwist : feed TwistWithCovarianceStamped (body-frame v + ω)
constexpr bool kUsePose  = true;
constexpr bool kUseTwist = true;

// ── 3. Output CSV path (one file per run) ─────────────────────────────────────
//   Overwrite freely — only one CSV is ever produced per run.
constexpr const char* kCsvPath = "/tmp/finenav_localizer_test.csv";

// ── 4. FineNavLocalizer filter parameters ────────────────────────────────────
constexpr double kUpdateRateHz        = 50.0;  ///< timer frequency      [Hz]
constexpr double kMaxDelaySec         = 0.3;   ///< max observation delay [s]
constexpr int    kPoseSmoothingSteps  = 10;     ///< pose obs reuse count
constexpr int    kTwistSmoothingSteps = 1;     ///< twist obs reuse count
constexpr double kPoseGateDist        = 1e9;   ///< Mahalanobis χ²(6) gate
constexpr double kTwistGateDist       = 1e9;
constexpr double kMaxDtSec            = 0.5;   ///< dt clamp              [s]
constexpr double kVelNoiseStd         = 0.5;   ///< process noise σ_v    [m/s/√s]
constexpr double kOmegaNoiseStd       = 0.1;   ///< process noise σ_ω  [rad/s/√s]

// ── 5. Observation noise ──────────────────────────────────────────────────────
constexpr double kPoseSigmaM      = 0.05;   ///< position σ             [m]
constexpr double kPoseYawSigmaRad = 0.02;   ///< yaw σ                  [rad]
constexpr double kTwistVelSigma   = 0.05;   ///< body-frame linear v σ  [m/s]
constexpr double kTwistOmegaSigma = 0.01;   ///< body-frame angular v σ [rad/s]

// ── 6. Observation delivery delay ────────────────────────────────────────────
//   Back-dates every header.stamp by a fixed amount to simulate sensor pipeline
//   latency (e.g. camera processing, network, serialisation).
//   The ESKF time-delay compensator (findDelayStep / updateWithDelay) corrects
//   for this automatically — no manual tuning at the filter level required.
//
//   Constraint: value must be < kMaxDelaySec, otherwise observations are
//   rejected as "too old" by timerCallback.
//   Set to 0.0 to disable (stamp = current wall time, no latency).
constexpr double kPoseDelaySec  = 0.005;   ///< fixed pose  pipeline delay [s]
constexpr double kTwistDelaySec = 0.005;   ///< fixed twist pipeline delay [s]

// ── 7. Scenario-specific simulation parameters ────────────────────────────────

// Scenarios 1 & 2 — uniform circle
constexpr double kCircleRadius   = 3.0;                        ///< [m]
constexpr double kCircleOmega    = 3.14159265358979323846/5.0; ///< ω [rad/s]
constexpr double kCircleDuration = 10.0;                       ///< sim time [s]
constexpr double kCirclePoseHz   = 10.0;   ///< pose observation rate  [Hz]
constexpr double kCircleTwistHz  = 20.0;   ///< twist observation rate [Hz]

// Scenario 2 — phase 5 (PoseJump): persistent step bias added to every pose obs.
// After t=8 s all pose observations carry this constant offset on top of Gaussian
// noise.  The filter does not know about the bias; observe how fast it tracks.
// If initial observations are rejected by the Mahalanobis gate (d² > kPoseGateDist),
// increase kPoseGateDist or reduce the step magnitude.
constexpr double kPoseJumpDx   = 2.0;    ///< step offset in x    [m]
constexpr double kPoseJumpDy   = 2.0;    ///< step offset in y    [m]
constexpr double kPoseJumpDyaw = 0.85;   ///< step offset in yaw  [rad]

// Scenario 3 — Lissajous figure-8  (x=A·sin(2ωt), y=A·sin(ωt))
constexpr double kFig8Amplitude = 3.0;
constexpr double kFig8Omega     = 3.14159265358979323846/5.0;
constexpr double kFig8Duration  = 20.0;
constexpr double kFig8PoseHz    = 10.0;
constexpr double kFig8TwistHz   = 20.0;

} // namespace cfg

```

---

### 📄 finenav_localizer/eval/eval_scenario.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//

// Scenarios (select via cfg::kScenario):
//   1 — Uniform circular motion, all ideal            [implemented]
//   2 — Uniform circular, Session-B stress events     [stub]
//   3 — High-dynamic Lissajous figure-8, ideal        [stub]
//
// CSV schema (extended — all 12 state dims + non-2D covariance diagonals):
//   t | gt_{x,y,yaw,vx,vy,wz} | est_{x,y,yaw,vx,vy,wz} |
//   est_{z,vz,wx,wy} | cov_{pz,rx,ry,vz,wx,wy} |
//   obs_{x,y,yaw,vx_body,wz_body} | has_pose_obs | has_twist_obs | scenario

#include "eval_config.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

#include "finenav_localizer/finenav_localizer.hpp"

using namespace std::chrono_literals;
namespace fn = finenav;

static_assert(cfg::kUsePose || cfg::kUseTwist,
    "cfg: at least one of kUsePose / kUseTwist must be true.");

// ═════════════════════════════════════════════════════════════════════════════
//  Data types
// ═════════════════════════════════════════════════════════════════════════════

/// Ground-truth state for 2-D planar motion.
struct GtState {
    double x{}, y{}, yaw{};
    double vx_world{}, vy_world{};  ///< world-frame linear velocity  [m/s]
    double vx_body{};               ///< body-frame forward speed      [m/s]
    double wz{};                    ///< body-frame yaw rate           [rad/s]
};

/// One logged row — extended schema covers all 12 state dims.
/// Non-2D covariance diagonals let us spot divergence in unused dimensions.
struct Row {
    double t{};

    // Ground truth (6 quantities)
    double gt_x{}, gt_y{}, gt_yaw{};
    double gt_vx{}, gt_vy{}, gt_wz{};

    // Estimate — primary 2-D dims (visualised as time-series)
    double est_x{}, est_y{}, est_yaw{};
    double est_vx{}, est_vy{}, est_wz{};   ///< world v_x, v_y;  body ω_z

    // Estimate — non-2D dims
    double est_z{}, est_vz{}, est_wx{}, est_wy{};

    // Error-state covariance diagonal for non-2D dims
    //   state layout: [ δp(0-2) | δθ(3-5) | δv(6-8) | δω(9-11) ]
    double cov_pz{};   ///< P[2,2]
    double cov_rx{};   ///< P[3,3]
    double cov_ry{};   ///< P[4,4]
    double cov_vz{};   ///< P[8,8]
    double cov_wx{};   ///< P[9,9]
    double cov_wy{};   ///< P[10,10]

    // Observations  (NaN = absent this tick)
    double obs_x{}, obs_y{}, obs_yaw{};       ///< pose
    double obs_vx_body{}, obs_wz_body{};      ///< twist (body frame)
    bool   has_pose_obs{false};
    bool   has_twist_obs{false};

    const char* scenario{""};
};

// ═════════════════════════════════════════════════════════════════════════════
//  Helper utilities  (file-internal linkage)
// ═════════════════════════════════════════════════════════════════════════════

static double noisy(double v, double sigma, std::mt19937& rng)
{
    return sigma > 0.0
        ? v + std::normal_distribution<double>(0.0, sigma)(rng)
        : v;
}

static double yawFromState(const fn::NavStateD& s)
{
    const Eigen::Quaterniond q(s.R.matrix());
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// Assemble pnode parameter overrides from cfg:: constants.
static rclcpp::NodeOptions makeOpts()
{
    rclcpp::NodeOptions o;
    o.append_parameter_override("update_rate_hz",        cfg::kUpdateRateHz);
    o.append_parameter_override("max_delay_sec",         cfg::kMaxDelaySec);
    o.append_parameter_override("pose_smoothing_steps",  int64_t{cfg::kPoseSmoothingSteps});
    o.append_parameter_override("twist_smoothing_steps", int64_t{cfg::kTwistSmoothingSteps});
    o.append_parameter_override("pose_gate_dist",        cfg::kPoseGateDist);
    o.append_parameter_override("twist_gate_dist",       cfg::kTwistGateDist);
    o.append_parameter_override("max_dt_sec",            cfg::kMaxDtSec);
    o.append_parameter_override("model.vel_noise_std",   cfg::kVelNoiseStd);
    o.append_parameter_override("model.omega_noise_std", cfg::kOmegaNoiseStd);
    return o;
}

/// Initialise the filter at the exact GT pose with tight uncertainty.
/// Velocity starts at zero with high covariance (filter learns from obs).
static void initLocalizer(fn::FineNavLocalizer& loc, const GtState& g,
                           const rclcpp::Node::SharedPtr& node)
{
    fn::FineNavLocalizer::PoseMsg m;
    m.header.stamp          = node->now();
    m.header.frame_id       = "map";
    m.pose.pose.position.x  = g.x;
    m.pose.pose.position.y  = g.y;
    m.pose.pose.position.z  = 0.0;
    m.pose.pose.orientation.w = std::cos(g.yaw * 0.5);
    m.pose.pose.orientation.x = 0.0;
    m.pose.pose.orientation.y = 0.0;
    m.pose.pose.orientation.z = std::sin(g.yaw * 0.5);
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0]  = 1e-4;   // x
    m.pose.covariance[7]  = 1e-4;   // y
    m.pose.covariance[35] = 1e-4;   // yaw
    loc.setInitialPose(m);
}

/// Build a noisy PoseWithCovarianceStamped from GT state.
static fn::FineNavLocalizer::PoseMsg buildPose(
    const GtState& g, double p_sigma, double yaw_sigma,
    const rclcpp::Time& stamp, std::mt19937& rng)
{
    const double yaw = noisy(g.yaw, yaw_sigma, rng);

    fn::FineNavLocalizer::PoseMsg m;
    m.header.stamp = (cfg::kPoseDelaySec > 0.0)
        ? stamp - rclcpp::Duration::from_seconds(cfg::kPoseDelaySec)
        : stamp;
    m.header.frame_id       = "map";
    m.pose.pose.position.x  = noisy(g.x, p_sigma, rng);
    m.pose.pose.position.y  = noisy(g.y, p_sigma, rng);
    m.pose.pose.position.z  = 0.0;
    m.pose.pose.orientation.w = std::cos(yaw * 0.5);
    m.pose.pose.orientation.x = 0.0;
    m.pose.pose.orientation.y = 0.0;
    m.pose.pose.orientation.z = std::sin(yaw * 0.5);
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0]  = std::max(p_sigma   * p_sigma,   1e-6);  // x
    m.pose.covariance[7]  = std::max(p_sigma   * p_sigma,   1e-6);  // y
    m.pose.covariance[14] = 1e-4;                                    // z
    m.pose.covariance[21] = 1e-4;                                    // roll
    m.pose.covariance[28] = 1e-4;                                    // pitch
    m.pose.covariance[35] = std::max(yaw_sigma * yaw_sigma, 1e-6);  // yaw
    return m;
}

/// Build a noisy TwistWithCovarianceStamped from GT state (body frame).
static fn::FineNavLocalizer::TwistMsg buildTwist(
    const GtState& g, double v_sigma, double w_sigma,
    const rclcpp::Time& stamp, std::mt19937& rng)
{
    fn::FineNavLocalizer::TwistMsg m;
    m.header.stamp = (cfg::kTwistDelaySec > 0.0)
        ? stamp - rclcpp::Duration::from_seconds(cfg::kTwistDelaySec)
        : stamp;
    m.header.frame_id       = "base_link";
    m.twist.twist.linear.x  = noisy(g.vx_body, v_sigma, rng);
    m.twist.twist.linear.y  = 0.0;
    m.twist.twist.linear.z  = 0.0;
    m.twist.twist.angular.x = 0.0;
    m.twist.twist.angular.y = 0.0;
    m.twist.twist.angular.z = noisy(g.wz, w_sigma, rng);
    m.twist.covariance.fill(0.0);
    m.twist.covariance[0]  = std::max(v_sigma * v_sigma, 1e-6);  // vx
    m.twist.covariance[7]  = 1e-4;                               // vy ~ 0
    m.twist.covariance[14] = 1e-4;                               // vz ~ 0
    m.twist.covariance[21] = 1e-4;                               // ωx ~ 0
    m.twist.covariance[28] = 1e-4;                               // ωy ~ 0
    m.twist.covariance[35] = std::max(w_sigma * w_sigma, 1e-6);  // wz
    return m;
}

/// Fill all estimate fields of a Row from the current filter state.
static void fillEstimate(Row& row, fn::FineNavLocalizer& loc)
{
    const fn::NavStateD              est = loc.getState();
    const fn::NavigationModelD::PMat P   = loc.getCovariance();

    // Primary 2-D dims.
    // getState() returns v in body frame; convert back to world frame so that
    // est_vx/vy are directly comparable with the world-frame GT columns.
    const Eigen::Vector3d v_world = est.R * est.v;
    row.est_x   = est.p.x();
    row.est_y   = est.p.y();
    row.est_yaw = yawFromState(est);
    row.est_vx  = v_world.x();   ///< world-frame vx  (matches gt_vx)
    row.est_vy  = v_world.y();   ///< world-frame vy  (matches gt_vy)
    row.est_wz  = est.omega.z();

    // Non-2D state values
    row.est_z   = est.p.z();
    row.est_vz  = est.v.z();
    row.est_wx  = est.omega.x();
    row.est_wy  = est.omega.y();

    // Non-2D covariance diagonals
    //   state layout: [ δp(0-2) | δθ(3-5) | δv(6-8) | δω(9-11) ]
    row.cov_pz = P(2,  2);
    row.cov_rx = P(3,  3);
    row.cov_ry = P(4,  4);
    row.cov_vz = P(8,  8);
    row.cov_wx = P(9,  9);
    row.cov_wy = P(10, 10);
}

/// Write all rows to cfg::kCsvPath.
static void writeCSV(const std::vector<Row>& rows)
{
    const std::string path = cfg::kCsvPath;
    std::ofstream f(path);
    if (!f) throw std::runtime_error("[writeCSV] Cannot open: " + path);

    f << "t,"
         "gt_x,gt_y,gt_yaw,gt_vx,gt_vy,gt_wz,"
         "est_x,est_y,est_yaw,est_vx,est_vy,est_wz,"
         "est_z,est_vz,est_wx,est_wy,"
         "cov_pz,cov_rx,cov_ry,cov_vz,cov_wx,cov_wy,"
         "obs_x,obs_y,obs_yaw,obs_vx_body,obs_wz_body,"
         "has_pose_obs,has_twist_obs,"
         "scenario\n";

    // Format a double: empty string for NaN (CSV-safe)
    auto F = [](double v) -> std::string {
        return std::isnan(v) ? "" : std::to_string(v);
    };

    for (const auto& r : rows) {
        f << r.t << ","
          << r.gt_x    << "," << r.gt_y    << "," << r.gt_yaw  << ","
          << r.gt_vx   << "," << r.gt_vy   << "," << r.gt_wz   << ","
          << r.est_x   << "," << r.est_y   << "," << r.est_yaw << ","
          << r.est_vx  << "," << r.est_vy  << "," << r.est_wz  << ","
          << r.est_z   << "," << r.est_vz  << ","
          << r.est_wx  << "," << r.est_wy  << ","
          << r.cov_pz  << "," << r.cov_rx  << "," << r.cov_ry  << ","
          << r.cov_vz  << "," << r.cov_wx  << "," << r.cov_wy  << ","
          << F(r.obs_x) << "," << F(r.obs_y) << "," << F(r.obs_yaw) << ","
          << F(r.obs_vx_body) << "," << F(r.obs_wz_body) << ","
          << (r.has_pose_obs  ? 1 : 0) << ","
          << (r.has_twist_obs ? 1 : 0) << ","
          << r.scenario << "\n";
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Scenario 1 — Uniform circular motion, all ideal
//
//  Ground truth (CCW circle):
//    x(t) =  R·cos(ω·t)           y(t) =  R·sin(ω·t)
//    yaw(t) = ω·t + π/2           (tangent direction)
//    vx_world = −R·ω·sin(ω·t)     vy_world = R·ω·cos(ω·t)
//    vx_body  =  R·ω  (const)     wz = ω  (const)
//
//  Observations (all at nominal noise — no outliers, no drop-outs):
//    Pose  @ cfg::kCirclePoseHz   (if cfg::kUsePose)
//    Twist @ cfg::kCircleTwistHz  (if cfg::kUseTwist)
// ═════════════════════════════════════════════════════════════════════════════
static void runCircleIdeal(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "═══ Scenario 1: Uniform circular, all ideal  "
        "(pose=%s  twist=%s) ═══",
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF");

    const double R = cfg::kCircleRadius;
    const double W = cfg::kCircleOmega;

    auto gt = [R, W](double t) -> GtState {
        const double th = W * t;
        GtState g;
        g.x        =  R * std::cos(th);
        g.y        =  R * std::sin(th);
        g.yaw      =  th + M_PI / 2.0;           // tangent (CCW)
        g.vx_world = -R * W * std::sin(th);
        g.vy_world =  R * W * std::cos(th);
        g.vx_body  =  R * W;                     // constant
        g.wz       =  W;
        return g;
    };

    fn::FineNavLocalizer localizer(node, makeOpts());
    initLocalizer(localizer, gt(0.0), node);
    rclcpp::sleep_for(40ms);   // let timer thread start

    constexpr double kDt    = 1.0 / cfg::kUpdateRateHz;
    constexpr double kTotal = cfg::kCircleDuration;
    const double     kNaN   = std::numeric_limits<double>::quiet_NaN();

    std::mt19937     rng(42);
    std::vector<Row> log;
    log.reserve(static_cast<std::size_t>(kTotal / kDt * 1.1));

    double t            = 0.0;
    double last_pose_t  = -999.0;
    double last_twist_t = -999.0;

    while (t < kTotal) {
        const GtState      g   = gt(t);
        const rclcpp::Time now = node->now();

        Row row;
        row.t             = t;
        row.gt_x          = g.x;
        row.gt_y          = g.y;
        row.gt_yaw        = g.yaw;
        row.gt_vx         = g.vx_world;
        row.gt_vy         = g.vy_world;
        row.gt_wz         = g.wz;
        row.obs_x         = kNaN;
        row.obs_y         = kNaN;
        row.obs_yaw       = kNaN;
        row.obs_vx_body   = kNaN;
        row.obs_wz_body   = kNaN;
        row.has_pose_obs  = false;
        row.has_twist_obs = false;
        row.scenario      = "CircleIdeal";

        // ── Pose observation ─────────────────────────────────────────────────
        if constexpr (cfg::kUsePose) {
            if ((t - last_pose_t) >= (1.0 / cfg::kCirclePoseHz)) {
                last_pose_t      = t;
                const auto pm    = buildPose(g, cfg::kPoseSigmaM,
                                             cfg::kPoseYawSigmaRad, now, rng);
                localizer.feedObservation(pm);
                row.has_pose_obs = true;
                row.obs_x        = pm.pose.pose.position.x;
                row.obs_y        = pm.pose.pose.position.y;
                row.obs_yaw      = std::atan2(
                    2.0 * pm.pose.pose.orientation.w * pm.pose.pose.orientation.z,
                    1.0 - 2.0 * pm.pose.pose.orientation.z
                               * pm.pose.pose.orientation.z);
            }
        }

        // ── Twist observation ────────────────────────────────────────────────
        if constexpr (cfg::kUseTwist) {
            if ((t - last_twist_t) >= (1.0 / cfg::kCircleTwistHz)) {
                last_twist_t      = t;
                const auto tm     = buildTwist(g, cfg::kTwistVelSigma,
                                               cfg::kTwistOmegaSigma, now, rng);
                localizer.feedObservation(tm);
                row.has_twist_obs = true;
                row.obs_vx_body   = tm.twist.twist.linear.x;
                row.obs_wz_body   = tm.twist.twist.angular.z;
            }
        }

        // ── Wait one filter tick, then snapshot ──────────────────────────────
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kDt)));

        fillEstimate(row, localizer);
        log.push_back(row);
        t += kDt;
    }

    writeCSV(log);
    RCLCPP_INFO(node->get_logger(),
        "[Scenario 1] %zu rows written → %s", log.size(), cfg::kCsvPath);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Scenario 2 — Uniform circular, Session-B stress events
//
//  GT: identical circle to Scenario 1 (R, ω from cfg::).
//
//  Phase structure  (cfg::kCircleDuration must be ≥ 10 s):
//    0– 2 s  Normal    pose @ cfg::kCirclePoseHz,  no outliers
//    2– 4 s  LowFreq   pose @ 1 Hz,                no outliers
//    4– 6 s  HighFreq  pose @ 30 Hz,               no outliers
//    6– 8 s  ObsJump   pose @ cfg::kCirclePoseHz,  30 % large-noise outliers
//                        (outlier: p_sigma=2 m, yaw_sigma=0.8 rad)
//    8–10 s  PoseJump  pose @ cfg::kCirclePoseHz,  nominal noise PLUS a
//                        persistent step bias (cfg::kPoseJumpDx/Dy/Dyaw).
//                        All obs after t=8 s are: GT + step + Gaussian noise.
//                        The filter does not know about the bias.
//
//  cfg::kUseTwist is respected in all phases (no twist-forced override).
// ═════════════════════════════════════════════════════════════════════════════
static void runCircleStress(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "═══ Scenario 2: Uniform circular, stress events  "
        "(pose=%s  twist=%s) ═══",
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF");

    const double R = cfg::kCircleRadius;
    const double W = cfg::kCircleOmega;

    auto gt = [R, W](double t) -> GtState {
        const double th = W * t;
        GtState g;
        g.x        =  R * std::cos(th);
        g.y        =  R * std::sin(th);
        g.yaw      =  th + M_PI / 2.0;
        g.vx_world = -R * W * std::sin(th);
        g.vy_world =  R * W * std::cos(th);
        g.vx_body  =  R * W;
        g.wz       =  W;
        return g;
    };

    // Phase descriptors.
    // pose_active : whether pose is fed (ANDed with cfg::kUsePose)
    // outlier_frac: fraction of pose obs that are gross outliers
    // bias_*      : persistent step offset added to every pose obs in this phase
    //               (simulates a sudden sensor bias — filter does not see it)
    struct Phase {
        double      t0, t1;
        const char* label;
        bool        pose_active;
        double      pose_hz;
        double      outlier_frac;
        double      bias_x, bias_y, bias_yaw;  ///< step offset on top of GT
    };
    static const Phase kPhases[] = {
        {0.0, 2.0,  "Normal",   true,  cfg::kCirclePoseHz, 0.00, 0.0, 0.0, 0.0},
        {2.0, 4.0,  "LowFreq",  true,  1.0,                0.00, 0.0, 0.0, 0.0},
        {4.0, 6.0,  "HighFreq", true,  30.0,               0.00, 0.0, 0.0, 0.0},
        {6.0, 8.0,  "ObsJump",  true,  cfg::kCirclePoseHz, 0.30, 0.0, 0.0, 0.0},
        {8.0, 10.0, "PoseJump", true,  cfg::kCirclePoseHz, 0.00,
            cfg::kPoseJumpDx, cfg::kPoseJumpDy, cfg::kPoseJumpDyaw},
    };
    auto find_phase = [&](double t) -> const Phase& {
        for (const auto& ph : kPhases)
            if (t >= ph.t0 && t < ph.t1) return ph;
        return kPhases[std::size(kPhases) - 1];
    };

    fn::FineNavLocalizer localizer(node, makeOpts());
    initLocalizer(localizer, gt(0.0), node);
    rclcpp::sleep_for(40ms);

    constexpr double kDt    = 1.0 / cfg::kUpdateRateHz;
    constexpr double kTotal = cfg::kCircleDuration;
    const double     kNaN   = std::numeric_limits<double>::quiet_NaN();

    std::mt19937     rng(42);
    std::vector<Row> log;
    log.reserve(static_cast<std::size_t>(kTotal / kDt * 1.1));

    double t            = 0.0;
    double last_pose_t  = -999.0;
    double last_twist_t = -999.0;

    while (t < kTotal) {
        const Phase&       ph  = find_phase(t);
        const GtState      g   = gt(t);
        const rclcpp::Time now = node->now();

        Row row;
        row.t             = t;
        row.gt_x          = g.x;
        row.gt_y          = g.y;
        row.gt_yaw        = g.yaw;
        row.gt_vx         = g.vx_world;
        row.gt_vy         = g.vy_world;
        row.gt_wz         = g.wz;
        row.obs_x         = kNaN;
        row.obs_y         = kNaN;
        row.obs_yaw       = kNaN;
        row.obs_vx_body   = kNaN;
        row.obs_wz_body   = kNaN;
        row.has_pose_obs  = false;
        row.has_twist_obs = false;
        row.scenario      = ph.label;

        // ── Pose observation ─────────────────────────────────────────────────
        const bool do_pose = cfg::kUsePose && ph.pose_active && ph.pose_hz > 0.0;
        if (do_pose && (t - last_pose_t) >= (1.0 / ph.pose_hz)) {
            last_pose_t = t;

            const bool   is_outlier =
                (ph.outlier_frac > 0.0) &&
                std::uniform_real_distribution<double>(0.0, 1.0)(rng) < ph.outlier_frac;
            const double p_sig   = is_outlier ? 2.0 : cfg::kPoseSigmaM;
            const double yaw_sig = is_outlier ? 0.8 : cfg::kPoseYawSigmaRad;

            // Apply persistent step bias to the observed pose (GT + bias + noise).
            // The filter's covariance (R in the message) still reflects nominal
            // noise — the filter has no knowledge of the bias.
            GtState g_obs = g;
            g_obs.x   += ph.bias_x;
            g_obs.y   += ph.bias_y;
            g_obs.yaw += ph.bias_yaw;

            const auto pm = buildPose(g_obs, p_sig, yaw_sig, now, rng);
            localizer.feedObservation(pm);
            row.has_pose_obs = true;
            row.obs_x        = pm.pose.pose.position.x;
            row.obs_y        = pm.pose.pose.position.y;
            row.obs_yaw      = std::atan2(
                2.0 * pm.pose.pose.orientation.w * pm.pose.pose.orientation.z,
                1.0 - 2.0 * pm.pose.pose.orientation.z
                           * pm.pose.pose.orientation.z);
        }

        // ── Twist observation (cfg::kUseTwist respected in all phases) ───────
        if (cfg::kUseTwist && (t - last_twist_t) >= (1.0 / cfg::kCircleTwistHz)) {
            last_twist_t      = t;
            const auto tm     = buildTwist(g, cfg::kTwistVelSigma,
                                           cfg::kTwistOmegaSigma, now, rng);
            localizer.feedObservation(tm);
            row.has_twist_obs = true;
            row.obs_vx_body   = tm.twist.twist.linear.x;
            row.obs_wz_body   = tm.twist.twist.angular.z;
        }

        // ── Wait one filter tick, then snapshot ──────────────────────────────
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kDt)));

        fillEstimate(row, localizer);
        log.push_back(row);
        t += kDt;
    }

    writeCSV(log);
    RCLCPP_INFO(node->get_logger(),
        "[Scenario 2] %zu rows written → %s", log.size(), cfg::kCsvPath);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Scenario 3 — High-dynamic Lissajous figure-8, all ideal
//
//  Ground truth:
//    x(t) = A·sin(2ω·t)       y(t) = A·sin(ω·t)
//
//  Exact analytic derivatives:
//    vx_world = 2A·ω·cos(2ω·t)         vy_world = A·ω·cos(ω·t)
//    yaw  = atan2(vy_world, vx_world)   (robot heading aligns with velocity)
//    wz   = (vx · vy' − vy · vx') / speed²
//         = 2ω·(2·cos(ω·t)·sin(2ω·t) − cos(2ω·t)·sin(ω·t))
//             / (4·cos²(2ω·t) + cos²(ω·t))
//    vx_body = speed = ‖v_world‖   (heading-aligned, vy_body ≡ 0)
//
//  Two complete figure-8 cycles in kFig8Duration = 20 s  (ω = π/5).
//  All observations at nominal noise — no outliers, no drop-outs.
// ═════════════════════════════════════════════════════════════════════════════
static void runHighDynIdeal(const rclcpp::Node::SharedPtr& node)
{
    RCLCPP_INFO(node->get_logger(),
        "═══ Scenario 3: High-dynamic Lissajous figure-8, ideal  "
        "(pose=%s  twist=%s) ═══",
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF");

    const double A = cfg::kFig8Amplitude;
    const double W = cfg::kFig8Omega;

    auto gt = [A, W](double t) -> GtState {
        const double wt      = W * t;
        const double cos_wt  = std::cos(wt);
        const double sin_wt  = std::sin(wt);
        const double cos_2wt = std::cos(2.0 * wt);
        const double sin_2wt = std::sin(2.0 * wt);

        const double vx_w = 2.0 * A * W * cos_2wt;
        const double vy_w = A * W * cos_wt;

        // wz = (vx · (dvy/dt) − vy · (dvx/dt)) / speed²
        //   dvx/dt = −4AW²·sin(2Wt)
        //   dvy/dt = −AW²·sin(Wt)
        const double speed_sq = vx_w * vx_w + vy_w * vy_w;
        const double d_vx = -4.0 * A * W * W * sin_2wt;
        const double d_vy =       -A * W * W * sin_wt;
        const double wz   = (speed_sq > 1e-12)
            ? (vx_w * d_vy - vy_w * d_vx) / speed_sq
            : 0.0;

        GtState g;
        g.x        = A * sin_2wt;
        g.y        = A * sin_wt;
        g.yaw      = std::atan2(vy_w, vx_w);
        g.vx_world = vx_w;
        g.vy_world = vy_w;
        g.vx_body  = std::sqrt(speed_sq);   // heading aligned → vy_body = 0
        g.wz       = wz;
        return g;
    };

    fn::FineNavLocalizer localizer(node, makeOpts());
    initLocalizer(localizer, gt(0.0), node);
    rclcpp::sleep_for(40ms);

    constexpr double kDt    = 1.0 / cfg::kUpdateRateHz;
    constexpr double kTotal = cfg::kFig8Duration;
    const double     kNaN   = std::numeric_limits<double>::quiet_NaN();

    std::mt19937     rng(42);
    std::vector<Row> log;
    log.reserve(static_cast<std::size_t>(kTotal / kDt * 1.1));

    double t            = 0.0;
    double last_pose_t  = -999.0;
    double last_twist_t = -999.0;

    while (t < kTotal) {
        const GtState      g   = gt(t);
        const rclcpp::Time now = node->now();

        Row row;
        row.t             = t;
        row.gt_x          = g.x;
        row.gt_y          = g.y;
        row.gt_yaw        = g.yaw;
        row.gt_vx         = g.vx_world;
        row.gt_vy         = g.vy_world;
        row.gt_wz         = g.wz;
        row.obs_x         = kNaN;
        row.obs_y         = kNaN;
        row.obs_yaw       = kNaN;
        row.obs_vx_body   = kNaN;
        row.obs_wz_body   = kNaN;
        row.has_pose_obs  = false;
        row.has_twist_obs = false;
        row.scenario      = "HighDynIdeal";

        // ── Pose observation ─────────────────────────────────────────────────
        if constexpr (cfg::kUsePose) {
            if ((t - last_pose_t) >= (1.0 / cfg::kFig8PoseHz)) {
                last_pose_t      = t;
                const auto pm    = buildPose(g, cfg::kPoseSigmaM,
                                             cfg::kPoseYawSigmaRad, now, rng);
                localizer.feedObservation(pm);
                row.has_pose_obs = true;
                row.obs_x        = pm.pose.pose.position.x;
                row.obs_y        = pm.pose.pose.position.y;
                row.obs_yaw      = std::atan2(
                    2.0 * pm.pose.pose.orientation.w * pm.pose.pose.orientation.z,
                    1.0 - 2.0 * pm.pose.pose.orientation.z
                               * pm.pose.pose.orientation.z);
            }
        }

        // ── Twist observation ────────────────────────────────────────────────
        if constexpr (cfg::kUseTwist) {
            if ((t - last_twist_t) >= (1.0 / cfg::kFig8TwistHz)) {
                last_twist_t      = t;
                const auto tm     = buildTwist(g, cfg::kTwistVelSigma,
                                               cfg::kTwistOmegaSigma, now, rng);
                localizer.feedObservation(tm);
                row.has_twist_obs = true;
                row.obs_vx_body   = tm.twist.twist.linear.x;
                row.obs_wz_body   = tm.twist.twist.angular.z;
            }
        }

        // ── Wait one filter tick, then snapshot ──────────────────────────────
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kDt)));

        fillEstimate(row, localizer);
        log.push_back(row);
        t += kDt;
    }

    writeCSV(log);
    RCLCPP_INFO(node->get_logger(),
        "[Scenario 3] %zu rows written → %s", log.size(), cfg::kCsvPath);
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_finenav_localizer");

    RCLCPP_INFO(node->get_logger(),
        "[test_finenav_localizer]  scenario=%d  pose=%s  twist=%s  → %s",
        cfg::kScenario,
        cfg::kUsePose  ? "ON" : "OFF",
        cfg::kUseTwist ? "ON" : "OFF",
        cfg::kCsvPath);

    switch (cfg::kScenario) {
        case 1:  runCircleIdeal(node);   break;
        case 2:  runCircleStress(node);  break;
        case 3:  runHighDynIdeal(node);  break;
        default:
            RCLCPP_ERROR(node->get_logger(),
                "Unknown cfg::kScenario=%d  (valid: 1, 2, 3).", cfg::kScenario);
            rclcpp::shutdown();
            return 1;
    }

    RCLCPP_INFO(node->get_logger(),
        "\n[Done]  Visualise with:\n"
        "  python3 finenav_localizer/eval/plot.py %s",
        cfg::kCsvPath);

    rclcpp::shutdown();
    return 0;
}
```

---

### 📄 finenav_localizer/eval/plot.py

```python
#!/usr/bin/env python3
"""Visualise FineNavLocalizer scenario-test output.

Usage:
    python3 eval/plot.py [CSV_PATH]

Default: /tmp/finenav_localizer_test.csv   (mirrors cfg::kCsvPath)

Extended CSV schema produced by test_finenav_localizer:
  t | gt_{x,y,yaw,vx,vy,wz} | est_{x,y,yaw,vx,vy,wz}
  | est_{z,vz,wx,wy} | cov_{pz,rx,ry,vz,wx,wy}
  | obs_{x,y,yaw,vx_body,wz_body} | has_pose_obs | has_twist_obs | scenario

Panel layout (3 rows × 3 cols):
  Row 0: 2-D trajectory        | x(t)    | y(t)
  Row 1: yaw(t)                | vx(t)   | vy(t)
  Row 2: wz(t)                 | non-2D covariance diagonals (spans 2 cols)

Observations are overlaid as scatter markers where present.
Scenario phases are shaded on every time-series panel.
Non-2D covariance diagonals (cov_pz, cov_rx, cov_ry, cov_vz, cov_wx, cov_wy)
are shown on a shared log-scale panel — runaway growth signals divergence in
the unused state dimensions of a 2-D scenario.

Requires: numpy, pandas, matplotlib ≥ 3.5
"""

#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import math
import os
import sys

import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D

# ── Default CSV path (mirrors cfg::kCsvPath in localizer_test_config.hpp) ─────
DEFAULT_CSV = "/tmp/finenav_localizer_test.csv"

# ── Scenario → colour (tab10).  Add new labels here when more scenarios land. ─
_KNOWN_SCENARIOS = [
    "CircleIdeal",                                            # Scenario 1
    "Normal", "LowFreq", "HighFreq", "ObsJump", "PoseJump",  # Scenario 2 phases
    "HighDynIdeal",                                           # Scenario 3
]
_PAL = plt.cm.tab10(np.linspace(0, 0.9, len(_KNOWN_SCENARIOS)))
_SC_COLOR = {sc: tuple(_PAL[i]) for i, sc in enumerate(_KNOWN_SCENARIOS)}


def _c(sc: str) -> tuple:
    """Colour for a scenario label (grey fallback for unknown labels)."""
    return _SC_COLOR.get(sc, (0.45, 0.45, 0.45, 1.0))


def _wrap(series):
    """Wrap angle series to (−π, π]."""
    return series.apply(lambda a: (a + math.pi) % (2 * math.pi) - math.pi)


# ─────────────────────────────────────────────────────────────────────────────
def _shade_phases(ax, df: pd.DataFrame, scenarios: list) -> None:
    """Draw semi-transparent phase bands and phase-boundary tick lines."""
    for sc in scenarios:
        m = df["scenario"] == sc
        if not m.any():
            continue
        t0 = df.loc[m, "t"].min()
        t1 = df.loc[m, "t"].max()
        ax.axvspan(t0, t1, alpha=0.08, color=_c(sc), zorder=0)
        ax.axvline(t0, color=_c(sc), lw=0.7, ls=":", alpha=0.6, zorder=1)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Time [s]", fontsize=8)


# ─────────────────────────────────────────────────────────────────────────────
def _ts(ax, t, gt_vals, est_vals, ylabel: str, title: str,
        obs_t=None, obs_vals=None, obs_label: str = "Obs",
        scenarios=None, df=None) -> None:
    """Standard time-series panel: GT line, estimate line, optional obs scatter."""
    ax.plot(t, gt_vals,  "k-",  lw=1.8, label="GT",  zorder=3)
    ax.plot(t, est_vals, "b--", lw=1.3, label="Est", zorder=4)
    if obs_t is not None and obs_vals is not None and len(obs_t) > 0:
        ax.scatter(obs_t, obs_vals,
                   color="tomato", s=16, zorder=6, alpha=0.80, label=obs_label)
    ax.set_ylabel(ylabel, fontsize=8)
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=6.5, loc="upper right")
    if scenarios is not None and df is not None:
        _shade_phases(ax, df, scenarios)


# ─────────────────────────────────────────────────────────────────────────────
def plot_csv(csv_path: str) -> None:
    df = pd.read_csv(csv_path)

    # Cast potentially-empty observation columns to float (NaN where absent)
    for col in ["obs_x", "obs_y", "obs_yaw", "obs_vx_body", "obs_wz_body"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df["has_pose_obs"]  = df["has_pose_obs"].astype(bool)
    df["has_twist_obs"] = df["has_twist_obs"].astype(bool)

    scenarios = list(df["scenario"].unique())
    t = df["t"].to_numpy()

    # ── Figure & GridSpec ────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 11))
    fig.suptitle(
        f"FineNavLocalizer  ▸  {os.path.basename(csv_path)}",
        fontsize=13, fontweight="bold", y=0.998)

    gs = gridspec.GridSpec(
        3, 3, figure=fig,
        hspace=0.55, wspace=0.38,
        left=0.055, right=0.975,
        top=0.955,  bottom=0.065)

    ax_traj = fig.add_subplot(gs[0, 0])
    ax_x    = fig.add_subplot(gs[0, 1])
    ax_y    = fig.add_subplot(gs[0, 2])
    ax_yaw  = fig.add_subplot(gs[1, 0])
    ax_vx   = fig.add_subplot(gs[1, 1])
    ax_vy   = fig.add_subplot(gs[1, 2])
    ax_wz   = fig.add_subplot(gs[2, 0])
    ax_cov  = fig.add_subplot(gs[2, 1:])   # spans columns 1 and 2

    # ── Convenience: filtered obs arrays ────────────────────────────────────
    pose_m  = df["has_pose_obs"]  & df["obs_x"].notna()
    twist_m = df["has_twist_obs"] & df["obs_wz_body"].notna()

    pose_t   = df.loc[pose_m,  "t"].to_numpy()
    twist_t  = df.loc[twist_m, "t"].to_numpy()

    # ── Panel 1: 2-D Trajectory ───────────────────────────────────────────────
    ax_traj.plot(df["gt_x"],  df["gt_y"],  "k-",  lw=2.0, zorder=3, label="GT")
    ax_traj.plot(df["est_x"], df["est_y"], "b--", lw=1.5, zorder=4, label="Est")
    ax_traj.plot(df["gt_x"].iloc[0], df["gt_y"].iloc[0],
                 "k^", ms=9, zorder=7, clip_on=False)

    for sc in scenarios:
        m = df["scenario"] == sc
        ax_traj.plot(df.loc[m, "est_x"], df.loc[m, "est_y"],
                     color=_c(sc), lw=2.2, alpha=0.80, zorder=5)
        pm = pose_m & m
        if pm.any():
            ax_traj.scatter(df.loc[pm, "obs_x"], df.loc[pm, "obs_y"],
                            color=_c(sc), s=10, zorder=6, alpha=0.55)

    ax_traj.set_aspect("equal")
    ax_traj.set_xlabel("X [m]", fontsize=8)
    ax_traj.set_ylabel("Y [m]", fontsize=8)
    ax_traj.set_title("2-D Trajectory", fontsize=9)
    ax_traj.grid(True, alpha=0.3)

    # Scenario patches + GT/Est entries in the traj legend
    traj_handles = [
        Line2D([0], [0], color="k", lw=2.0,            label="GT"),
        Line2D([0], [0], color="b", lw=1.5, ls="--",   label="Est"),
    ]
    for sc in scenarios:
        traj_handles.append(mpatches.Patch(color=_c(sc), alpha=0.75, label=sc))
    ax_traj.legend(handles=traj_handles, fontsize=6.5, loc="best")

    # ── Panel 2: x(t) ────────────────────────────────────────────────────────
    _ts(ax_x, t,
        df["gt_x"].to_numpy(), df["est_x"].to_numpy(),
        "x [m]", "x  (world frame)",
        obs_t=pose_t, obs_vals=df.loc[pose_m, "obs_x"].to_numpy(),
        obs_label="Pose obs",
        scenarios=scenarios, df=df)

    # ── Panel 3: y(t) ────────────────────────────────────────────────────────
    _ts(ax_y, t,
        df["gt_y"].to_numpy(), df["est_y"].to_numpy(),
        "y [m]", "y  (world frame)",
        obs_t=pose_t, obs_vals=df.loc[pose_m, "obs_y"].to_numpy(),
        obs_label="Pose obs",
        scenarios=scenarios, df=df)

    # ── Panel 4: yaw(t) ───────────────────────────────────────────────────────
    _ts(ax_yaw, t,
        _wrap(df["gt_yaw"]).to_numpy(), _wrap(df["est_yaw"]).to_numpy(),
        "yaw [rad]", "yaw  (world frame)",
        obs_t=pose_t if df["has_pose_obs"].any() and df["obs_yaw"].notna().any() else None,
        obs_vals=_wrap(df.loc[pose_m, "obs_yaw"]).to_numpy() if pose_m.any() else None,
        obs_label="Pose obs",
        scenarios=scenarios, df=df)

    # ── Panel 5: vx(t)  — world frame  ───────────────────────────────────────
    _ts(ax_vx, t,
        df["gt_vx"].to_numpy(), df["est_vx"].to_numpy(),
        "vx [m/s]", "vx  (world frame, velocity highlight)",
        scenarios=scenarios, df=df)

    # ── Panel 6: vy(t)  — world frame ────────────────────────────────────────
    _ts(ax_vy, t,
        df["gt_vy"].to_numpy(), df["est_vy"].to_numpy(),
        "vy [m/s]", "vy  (world frame, velocity highlight)",
        scenarios=scenarios, df=df)

    # ── Panel 7: wz(t)  — body frame ─────────────────────────────────────────
    _ts(ax_wz, t,
        df["gt_wz"].to_numpy(), df["est_wz"].to_numpy(),
        "ωz [rad/s]", "ωz  (body frame, velocity highlight)",
        obs_t=twist_t if twist_m.any() else None,
        obs_vals=df.loc[twist_m, "obs_wz_body"].to_numpy() if twist_m.any() else None,
        obs_label="Twist obs",
        scenarios=scenarios, df=df)

    # ── Panel 8: Non-2D covariance diagonals ──────────────────────────────────
    # state layout: [ δp(0-2) | δθ(3-5) | δv(6-8) | δω(9-11) ]
    # 2-D motion uses p_x/y, θ_z, v_x/y, ω_z.
    # The six unused dims are p_z, θ_x, θ_y, v_z, ω_x, ω_y (indices 2,3,4,8,9,10).
    COV_SPEC = [
        ("cov_pz", "p_z   (P[2,2])",   "#1f77b4"),
        ("cov_rx", "θ_x   (P[3,3])",   "#ff7f0e"),
        ("cov_ry", "θ_y   (P[4,4])",   "#2ca02c"),
        ("cov_vz", "v_z   (P[8,8])",   "#d62728"),
        ("cov_wx", "ω_x   (P[9,9])",   "#9467bd"),
        ("cov_wy", "ω_y  (P[10,10])",  "#8c564b"),
    ]
    for col, label, color in COV_SPEC:
        vals = np.maximum(df[col].to_numpy(), 1e-15)   # guard against log(0)
        ax_cov.plot(t, vals, label=label, color=color, lw=1.25, alpha=0.90)

    ax_cov.set_yscale("log")
    ax_cov.set_ylabel("Covariance  [unit²]", fontsize=8)
    ax_cov.set_title(
        "Non-2D error-state covariance diagonals\n"
        "p_z | θ_x | θ_y | v_z | ω_x | ω_y — runaway growth = divergence",
        fontsize=9)
    ax_cov.legend(fontsize=7.5, loc="upper right", ncol=2)
    _shade_phases(ax_cov, df, scenarios)

    # ── Save ──────────────────────────────────────────────────────────────────
    out_png = csv_path.replace(".csv", ".png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[plot_localizer]  {len(df)} rows  →  {out_png}")
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    if not os.path.exists(path):
        print(f"[ERROR] CSV not found: {path}")
        print("  Run the test first:")
        print("    ros2 run finenav_core test_finenav_localizer")
        sys.exit(1)
    plot_csv(path)


if __name__ == "__main__":
    main()

```

---

### 📄 finenav_localizer/include/finenav_localizer/eskf.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/logging.hpp>

#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/kalman_filter/time_delay_kalman_filter.hpp"

namespace finenav {

// ============================================================================
//  ESKF<Model>  —  Error-State Kalman Filter with time-delay compensation
//
//  Internally, the error-state covariance is maintained inside a
//  TimeDelayKalmanFilter, which expands the state to:
//
//      x_ex = [ δx_k | δx_{k-1} | … | δx_{k-d+1} ]
//
//  This lets delayed observations update the covariance and nominal state
//  correctly via cross-covariance blocks, without storing past nominal states.
//
//  ── Template parameter `Model` must expose ─────────────────────────────────
//    typename Model::ScalarT          —  scalar type (float / double)
//    typename Model::NavStateT        —  nominal state struct
//    typename Model::ErrorVec         —  error-state column vector (kNavStateDim × 1)
//    typename Model::FMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::QMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::PMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::Options          —  noise-parameter struct
//
//    Model::predictNominal(s, dt)     —  propagate nominal state forward
//    Model::computeF(s, dt)           —  error-state transition Jacobian F
//    Model::buildQ(opts, dt)          —  discrete process-noise matrix Q
//    Model::applyErrorState(s, dx)    —  nominal ← nominal ⊕ dx
//    Model::projectCovariance(P, dx)  —  ESKF covariance reset projection
//
//  ── Delay-handling design ──────────────────────────────────────────────────
//  • Every predict(dt) call:
//      1. Saves the PRE-propagation nominal into nominal_history_ (pushed to front).
//         nominal_history_[0] = nominal one tick ago  (delay_step 1)
//         nominal_history_[d-1] = nominal d ticks ago (delay_step d)
//      2. Propagates the nominal state forward.
//      3. Slides the TimeDelayKF extended covariance and accumulates delay times.
//
//  • findDelayStep(delay_sec) converts a wall-clock delay to the closest
//    integer delay slot index to pass to update(..., delay_step).
//
//  • getStateAtDelay(d) returns nominal_history_[d-1], which is the nominal
//    state at the time of capture of a delay_step=d observation.
//    This is critical for computing correct innovations and Jacobians.
//
//  • update<ObsDim>(..., delay_step) calls updateWithDelay internally.
//    After the KF update:
//      1. dx_0 ← getLatestX()  (block 0 = current-time error correction)
//      2. nominal ← applyErrorState(nominal, dx_0)
//      3. For each historical slot i ≥ 1:
//           dx_i ← getXAtSlot(i)  (past error correction from cross-covariance)
//           nominal_history_[i−1] ← applyErrorState(history[i−1], dx_i)
//         This keeps the nominal history consistent with the TDKF posterior,
//         so future delayed observations compute correct innovations.
//      4. P(0,0) ← projectCovariance(P(0,0), dx_0)   [leading block only]
//      5. resetState() — zero all error-state slots in tdkf_
//
//  ── Thread safety ──────────────────────────────────────────────────────────
//  Not thread-safe.  All calls must be serialized by the owner
//  (FineNavLocalizer's timerCallback holds filter_mutex_).
//
//  This file is header-only (templates).
// ============================================================================
template <typename Model>
class ESKF {
public:
    using S         = typename Model::ScalarT;
    using NavStateT = typename Model::NavStateT;
    using ErrorVec  = typename Model::ErrorVec;
    using FMat      = typename Model::FMat;
    using QMat      = typename Model::QMat;
    using PMat      = typename Model::PMat;

    static constexpr int kDim = kNavStateDim;  ///< 12

    // ---- Construction -----------------------------------------------------

    /// @param model_opts      Process-noise parameters forwarded to Model::buildQ.
    /// @param max_delay_step  Number of delay slots (≥ 1).
    ///                        Typically ceil(max_delay_sec * update_rate_hz).
    ///                        Observations older than this many ticks are rejected.
    /// @param init_cov        Initial state covariance P₀.
    explicit ESKF(typename Model::Options model_opts   = {},
                  int                    max_delay_step = 50,
                  PMat                   init_cov       = PMat::Identity() * static_cast<S>(1e-2))
        : model_opts_(std::move(model_opts))
        , accumulated_delay_times_(static_cast<std::size_t>(max_delay_step), 1.0e15)
        , logger_(rclcpp::get_logger("finenav.ESKF"))
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, init_cov.template cast<double>(), max_delay_step);
    }

    // ---- Predict step -----------------------------------------------------
    //
    //  Nominal-state propagation + error-state covariance propagation.
    //
    //  Since the error state is always reset to zero after every update(),
    //  x_next passed to predictWithDelay is the zero vector.  Only the
    //  covariance (and its cross-correlation blocks) advances in time.
    //
    //  After predict(), accumulateDelayTime(dt) updates accumulated_delay_times_
    //  so that findDelayStep() returns meaningful indices.
    void predict(S dt)
    {
        if (dt <= static_cast<S>(0.0)) {
            RCLCPP_WARN(logger_,
                "[ESKF] predict() called with non-positive dt=%.6f — skipping.",
                static_cast<double>(dt));
            return;
        }

        // Compute F and Q from the current nominal state *before* propagating.
        const FMat F = Model::computeF(nominal_, static_cast<S>(dt));
        const QMat Q = Model::buildQ(model_opts_, static_cast<S>(dt));

        // ── Save nominal into history BEFORE propagation ────────────────────
        //  nominal_history_[0] will become delay_step=1 (one tick ago).
        //  The oldest slot beyond max_delay_step is discarded.
        nominal_history_.push_front(nominal_);
        if (static_cast<int>(nominal_history_.size()) >= tdkf_.maxDelayStep()) {
            nominal_history_.pop_back();
        }

        // Propagate nominal state.
        nominal_ = Model::predictNominal(nominal_, static_cast<S>(dt));

        // Propagate error-state covariance via time-delay extended structure.
        // x_next is zero: error state is always reset after each update().
        const Eigen::MatrixXd x_next = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.predictWithDelay(x_next,
                               F.template cast<double>(),
                               Q.template cast<double>());

        // Track elapsed time in each delay slot.
        accumulateDelayTime(static_cast<double>(dt));

        RCLCPP_DEBUG(logger_,
            "[ESKF] predict  dt=%.4f  |p|=%.3f  |v|=%.3f  |ω|=%.3f",
            static_cast<double>(dt),
            static_cast<double>(nominal_.p.norm()),
            static_cast<double>(nominal_.v.norm()),
            static_cast<double>(nominal_.omega.norm()));
    }

    // ---- Generic observation update  (compile-time observation dimension) -
    //
    //  @tparam ObsDim    Observation vector length.
    //  @param innovation  z − h(x̂)                       (ObsDim × 1)
    //  @param H           Observation Jacobian             (ObsDim × kDim)
    //  @param R_noise     Observation-noise covariance     (ObsDim × ObsDim)
    //  @param delay_step  Delay slot (0 = no delay, from findDelayStep()).
    //                     Default 0 is backward-compatible with callers that
    //                     do not yet pass a delay.
    //
    //  Returns false if updateWithDelay rejects the observation (e.g.
    //  delay_step ≥ max_delay_step) or if the KF gain is degenerate.
    template <int ObsDim>
    bool update(const Eigen::Matrix<S, ObsDim, 1>      & innovation,
                const Eigen::Matrix<S, ObsDim, kDim>   & H,
                const Eigen::Matrix<S, ObsDim, ObsDim> & R_noise,
                int                                      delay_step = 0)
    {
        // ── 1. Delayed KF update ────────────────────────────────────────────
        if (!tdkf_.updateWithDelay(innovation.template cast<double>(),
                                   H.template cast<double>(),
                                   R_noise.template cast<double>(),
                                   delay_step)) {
            return false;
        }

        // ── 2. Extract error-state correction from the leading block ────────
        const ErrorVec dx = tdkf_.getLatestX().template cast<S>();
        const S dx_norm   = dx.norm();  // capture for debug log

        // ── 3. Apply error corrections to ALL nominal states ────────────────
        //
        //  dx_0  (slot 0): correction for current nominal.
        //  dx_i  (slot i, i ≥ 1): correction for nominal_history_[i−1].
        //    These arise from the cross-covariance blocks that propagate the
        //    measurement's information back through the delay.  Applying them
        //    keeps the history consistent with the TDKF posterior, so that a
        //    future observation at the same delay slot sees the correct
        //    innovation (z − h(corrected_historical_nominal)) and does not
        //    receive an inflated correction from a stale linearisation point.
        nominal_ = Model::applyErrorState(nominal_, dx);

        const int n_hist = std::min(
            tdkf_.maxDelayStep() - 1,
            static_cast<int>(nominal_history_.size()));
        for (int i = 0; i < n_hist; ++i) {
            // TDKF slot index (i+1) ↔ nominal_history_[i]
            const ErrorVec dx_i =
                tdkf_.getXAtSlot(i + 1).template cast<S>();
            // Only bother if the correction is non-trivial (avoids unnecessary
            // SO3 exponentials when the cross-covariance contribution is zero).
            if (dx_i.squaredNorm() > static_cast<S>(1e-20)) {
                nominal_history_[static_cast<std::size_t>(i)] =
                    Model::applyErrorState(nominal_history_[static_cast<std::size_t>(i)],
                                          dx_i);
            }
        }

        // ── 4. ESKF covariance projection on the leading block ──────────────
        //  P(0,0) ← G · P(0,0) · Gᵀ   where G = I − 0.5·[δθ]×  (SO3 block)
        //  Cross-correlation blocks P(i,0) / P(0,j) for i,j ≥ 1 are not
        //  explicitly projected because G ≈ I for small δθ (second-order).
        const PMat P_lead      = tdkf_.getLatestP().template cast<S>();
        const PMat P_projected = Model::projectCovariance(P_lead, dx);
        tdkf_.setLeadingCovariance(P_projected.template cast<double>());

        // ── 5. Reset all error-state slots to zero (ESKF reset step) ────────
        tdkf_.resetState();

        RCLCPP_DEBUG(logger_,
            "[ESKF] update  ObsDim=%d  delay_step=%d  |innov|=%.4f  |dx|=%.4f",
            ObsDim, delay_step,
            static_cast<double>(innovation.norm()),
            static_cast<double>(dx_norm));
        return true;
    }

    // ---- Delay-step helpers (mirrors autoware EKFModule) ------------------

    /// Find the delay-slot index whose accumulated time is closest to
    /// delay_time_sec.  Returns accumulated_delay_times_.size() if
    /// delay_time_sec exceeds the maximum tracked delay (caller should
    /// reject the observation).
    [[nodiscard]] std::size_t findDelayStep(double delay_time_sec) const
    {
        // If larger than the oldest slot, signal rejection.
        if (delay_time_sec > accumulated_delay_times_.back()) {
            return accumulated_delay_times_.size();
        }

        auto lower = std::lower_bound(
            accumulated_delay_times_.begin(),
            accumulated_delay_times_.end(),
            delay_time_sec);

        if (lower == accumulated_delay_times_.begin()) {
            return 0u;
        }
        if (lower == accumulated_delay_times_.end()) {
            return accumulated_delay_times_.size() - 1u;
        }

        // Pick whichever neighbour is closer.
        const auto prev = lower - 1;
        const bool closer_to_prev =
            (delay_time_sec - *prev) < (*lower - delay_time_sec);

        return closer_to_prev
            ? static_cast<std::size_t>(std::distance(accumulated_delay_times_.begin(), prev))
            : static_cast<std::size_t>(std::distance(accumulated_delay_times_.begin(), lower));
    }

    /// Maximum delay time currently tracked (seconds).
    /// Observations older than this should be discarded.
    [[nodiscard]] double maxTrackedDelaySec() const
    {
        return accumulated_delay_times_.empty() ? 0.0 : accumulated_delay_times_.back();
    }

    // ---- Accessors --------------------------------------------------------
    [[nodiscard]] const NavStateT & getState() const { return nominal_; }

    /// Get the nominal state at a specific delay step.
    ///   delay_step = 0  →  current nominal (same as getState())
    ///   delay_step = d  →  nominal from d ticks ago (= nominal_history_[d−1])
    ///
    /// Use this to compute the innovation for a delayed observation:
    ///   innov = z − h(getStateAtDelay(delay_step))
    ///
    /// Falls back to the oldest available entry if d exceeds history depth.
    [[nodiscard]] const NavStateT & getStateAtDelay(int delay_step) const
    {
        if (delay_step <= 0 || nominal_history_.empty()) {
            return nominal_;
        }
        const auto idx = static_cast<std::size_t>(delay_step - 1);
        if (idx < nominal_history_.size()) {
            return nominal_history_[idx];
        }
        return nominal_history_.back();  // oldest available
    }

    /// Returns the leading block of the extended covariance — i.e. the
    /// covariance of the CURRENT (latest) error-state estimate.
    [[nodiscard]] PMat getCovariance() const
    {
        return tdkf_.getLatestP().template cast<S>();
    }

    // ---- Mutators ---------------------------------------------------------

    void setState(const NavStateT & state) { nominal_ = state; }
    void setState(NavStateT       && state) { nominal_ = std::move(state); }

    /// Overwrites the full extended covariance (all delay slots) with P.
    /// Also resets accumulated_delay_times_ to 1e15 and clears nominal history.
    /// Use this for a full initialisation / reinit after setInitialPose.
    void setCovariance(const PMat & P)
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, P.template cast<double>(), tdkf_.maxDelayStep());
        accumulated_delay_times_.assign(
            static_cast<std::size_t>(tdkf_.maxDelayStep()), 1.0e15);
        nominal_history_.clear();
    }

    /// Convenience: reset nominal state AND full extended covariance atomically.
    void reinitialize(const NavStateT & state, const PMat & P)
    {
        nominal_ = state;
        setCovariance(P);  // also clears nominal_history_
    }

    [[nodiscard]] const typename Model::Options & getModelOptions() const { return model_opts_; }
    void setModelOptions(const typename Model::Options & opts)             { model_opts_ = opts; }

    [[nodiscard]] int maxDelayStep() const { return tdkf_.maxDelayStep(); }

private:
    // ---- Internal helpers -------------------------------------------------

    /// Shift accumulated_delay_times_ one slot to the right, insert 0.0 at
    /// slot 0, and add dt to all existing slots.  Mirrors autoware's
    /// EKFModule::accumulate_delay_time().
    void accumulateDelayTime(double dt)
    {
        std::copy_backward(
            accumulated_delay_times_.begin(),
            accumulated_delay_times_.end() - 1,
            accumulated_delay_times_.end());

        accumulated_delay_times_.front() = 0.0;
        for (std::size_t i = 1u; i < accumulated_delay_times_.size(); ++i) {
            accumulated_delay_times_[i] += dt;
        }
    }

    // ---- Data members -----------------------------------------------------
    NavStateT               nominal_{};        ///< nominal (best-estimate) state
    /// Circular history of pre-propagation nominal states.
    /// nominal_history_[0] = nominal one tick ago  (delay_step = 1)
    /// nominal_history_[d-1] = nominal d ticks ago (delay_step = d)
    /// Size is bounded to maxDelayStep()-1 by predict().
    std::deque<NavStateT, Eigen::aligned_allocator<NavStateT>>   nominal_history_;
    TimeDelayKalmanFilter   tdkf_;             ///< extended error-state covariance
    typename Model::Options model_opts_;       ///< process-noise parameters
    std::vector<double>     accumulated_delay_times_;  ///< slot → elapsed time [s]
    rclcpp::Logger          logger_;
};

// ---- Convenience aliases --------------------------------------------------
using ESKFd = ESKF<NavigationModel<double>>;
using ESKFf = ESKF<NavigationModel<float>>;

}  // namespace finenav

```

---

### 📄 finenav_localizer/include/finenav_localizer/filter_model.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace finenav {

// ============================================================================
//  Dimension of the error-state vector: [δp(3) | δθ(3) | δv(3) | δω(3)]
// ============================================================================
inline constexpr int kNavStateDim = 12;

// ============================================================================
//  NavState<S>  —  nominal state for the [p, R, v, ω] navigation model
//
//  All quantities are expressed in the *world* frame unless noted otherwise.
//  Angular velocity ω is the body-frame angular rate (same convention as IMU).
// ============================================================================
template <typename S = double>
struct NavState {
    // Sophus::SO3<S> internally holds an Eigen::Quaternion<S> which may
    // require 16-byte (SSE) or 32-byte (AVX) alignment.  When NavState is
    // stored in heap containers (std::deque, std::vector) the allocator must
    // honour this requirement; the macro below ensures it does.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SO3T  = Sophus::SO3<S>;
    using Vec3T = Eigen::Matrix<S, 3, 1>;

    Vec3T p     = Vec3T::Zero();   ///< position  (world frame)
    SO3T  R;                       ///< rotation  world ← body  (identity by default)
    Vec3T v     = Vec3T::Zero();   ///< linear velocity  (world frame)
    Vec3T omega = Vec3T::Zero();   ///< angular velocity (body frame)

    NavState() = default;
    NavState(Vec3T position, SO3T rotation, Vec3T velocity, Vec3T ang_vel)
        : p(std::move(position))
        , R(std::move(rotation))
        , v(std::move(velocity))
        , omega(std::move(ang_vel))
    {}
};

using NavStateF = NavState<float>;
using NavStateD = NavState<double>;

// ============================================================================
//  NavigationModel<S>  —  constant-velocity dead-reckoning model
//
//  Nominal-state propagation over dt:
//      p  ←  p + v · dt
//      R  ←  R · Exp(ω · dt)
//      v  ←  v               (no external acceleration)
//      ω  ←  ω               (no external angular acceleration)
//
//  Error-state layout (right-perturbation, 12-dim):
//      [ δp(3) | δθ(3) | δv(3) | δω(3) ]
//
//  The class exposes only static methods so that it acts as a pure
//  "model descriptor" consumed by ESKF<Model>.
// ============================================================================
template <typename S = double>
class NavigationModel {
public:
    // ---- Exported type aliases (required by ESKF) --------------------------
    using ScalarT   = S;
    using SO3T      = Sophus::SO3<S>;
    using Vec3T     = Eigen::Matrix<S, 3, 1>;
    using Mat3T     = Eigen::Matrix<S, 3, 3>;
    using NavStateT = NavState<S>;
    using ErrorVec  = Eigen::Matrix<S, kNavStateDim, 1>;
    using FMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< state-transition Jacobian
    using QMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< process-noise covariance
    using PMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< state covariance

    // ---- Process-noise configuration --------------------------------------
    struct Options { // Since we use a contant-velocity model, process error comes from variation of velocity
        S vel_noise_std   = static_cast<S>(0.10);  ///< lin-velocity noise σ         [m/s/√s]
        S omega_noise_std = static_cast<S>(0.10);  ///< ang-velocity noise σ         [rad/s/√s]
    };

    // ---- Nominal-state propagation ----------------------------------------
    [[nodiscard]]
    static NavStateT predictNominal(const NavStateT& s, S dt) {
        NavStateT ns;
        ns.p     = s.p + s.v * dt;
        ns.R     = s.R * SO3T::exp(s.omega * dt);
        ns.v     = s.v;
        ns.omega = s.omega;
        return ns;
    }

    // ---- Error-state transition Jacobian  F  (12×12)  ---------------------
    //
    //  Derived from first-order perturbation of the nominal dynamics:
    //    δp' = δp + δv·dt
    //    δθ' ≈ Exp(−ω·dt)·δθ + I·dt·δω     (right-perturbation model;
    //                                         Jr(ω·dt) ≈ I for small ω·dt)
    //    δv' = δv
    //    δω' = δω
    //
    [[nodiscard]]
    static FMat computeF(const NavStateT& s, S dt) {
        FMat F = FMat::Identity();

        // ∂δp / ∂δv
        F.template block<3, 3>(0, 6) = Mat3T::Identity() * dt;

        // ∂δθ / ∂δθ  —  Exp(−ω·dt): tangent transported by the rotation increment
        F.template block<3, 3>(3, 3) = SO3T::exp(-s.omega * dt).matrix();

        // ∂δθ / ∂δω  —  first-order: Jr(ω·dt)·dt ≈ I·dt
        F.template block<3, 3>(3, 9) = Mat3T::Identity() * dt;

        return F;
    }

    // ---- Discrete process-noise covariance  Q  (12×12) -------------------
    [[nodiscard]]
    static QMat buildQ(const Options& opts, S dt) {
        QMat Q = QMat::Zero();
        const S dt2 = dt * dt;

        auto diag3 = [](S sigma_sq_dt2) -> Mat3T {
            return Mat3T::Identity() * sigma_sq_dt2;
        };

        Q.template block<3, 3>(0, 0) = Mat3T::Zero(); // no direct position noise in constant-velocity model
        Q.template block<3, 3>(3, 3) = Mat3T::Zero(); // no direct rotation noise in constant-velocity model
        Q.template block<3, 3>(6, 6) = diag3(opts.vel_noise_std   * opts.vel_noise_std   * dt);
        Q.template block<3, 3>(9, 9) = diag3(opts.omega_noise_std * opts.omega_noise_std * dt);

        return Q;
    }

    // ---- Apply error-state correction to nominal state --------------------
    //  After correction the caller is responsible for zeroing dx.
    [[nodiscard]]
    static NavStateT applyErrorState(const NavStateT& nominal, const ErrorVec& dx) {
        NavStateT corrected;
        corrected.p     = nominal.p     + dx.template segment<3>(0);
        corrected.R     = nominal.R     * SO3T::exp(dx.template segment<3>(3));
        corrected.v     = nominal.v     + dx.template segment<3>(6);
        corrected.omega = nominal.omega + dx.template segment<3>(9);
        return corrected;
    }

    // ---- Covariance projection after error-state correction ---------------
    //  P ← J·P·J^T,   J = I − 0.5·[δθ]×  in the (3,3) rotation block.
    //  Ref: ESKF textbook eq. (3.63).
    [[nodiscard]]
    static PMat projectCovariance(const PMat& P, const ErrorVec& dx) {
        FMat J = FMat::Identity();
        J.template block<3, 3>(3, 3) =
            Mat3T::Identity() - static_cast<S>(0.5) * SO3T::hat(dx.template segment<3>(3));
        return J * P * J.transpose();
    }

    /// Typed aggregate returned by every observation helper.
    template <int Dim>
    struct ObsData {
        Eigen::Matrix<S, Dim, kNavStateDim> H;           ///< observation Jacobian
        Eigen::Matrix<S, Dim, 1>            innovation;  ///< z − h(x̂)
    };

    // ---- Observe 6-DOF pose  [p_world, R_world←body]  --------------------
    //
    //  h_p(x) = p,    ∂h_p/∂δp = I₃
    //  h_θ(x) = δθ,   ∂h_θ/∂δθ = I₃   (linearised via log map)
    //
    //  H (6 × 12):
    //    [ I₃   0    0   0 ]   ← position rows
    //    [ 0    I₃   0   0 ]   ← rotation rows
    [[nodiscard]]
    static ObsData<6> observePose(const NavStateT& nominal,
                                  const Vec3T&     pos_obs,
                                  const SO3T&      rot_obs)
    {
        Eigen::Matrix<S, 6, kNavStateDim> H = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        H.template block<3, 3>(0, 0) = Mat3T::Identity();  // ∂z_p / ∂δp
        H.template block<3, 3>(3, 3) = Mat3T::Identity();  // ∂z_θ / ∂δθ

        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = pos_obs - nominal.p;
        innov.template tail<3>() = (nominal.R.inverse() * rot_obs).log();

        return {H, innov};
    }

    // ---- Observe body-frame twist  [v_body, ω_body]  ----------------------
    //
    //  Both velocity and angular velocity are in the *body* frame.
    //  State stores v in world frame, so the observation function is:
    //    h_v(x) = R^T · v
    //    h_ω(x) = ω
    //
    //  First-order Jacobian (right-perturbation  R = R̂·Exp(δθ)):
    //    ∂h_v / ∂δθ = [R̂^T · v̂]×   (skew of nominal body-frame velocity)
    //    ∂h_v / ∂δv = R̂^T
    //    ∂h_ω / ∂δω = I₃
    //
    //  H (6 × 12):
    //    [ 0   [R̂ᵀv̂]×   R̂ᵀ   0  ]   ← v_body rows
    //    [ 0   0          0    I₃ ]   ← ω_body rows
    [[nodiscard]]
    static ObsData<6> observeVelocity(const NavStateT& nominal,
                                      const Vec3T&     lin_vel_obs_body,
                                      const Vec3T&     ang_vel_obs_body)
    {
        const Mat3T Rt         = nominal.R.inverse().matrix();  // R̂^T
        const Vec3T v_body_hat = Rt * nominal.v;                // R̂^T · v̂

        Eigen::Matrix<S, 6, kNavStateDim> H = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        H.template block<3, 3>(0, 3) = SO3T::hat(v_body_hat);  // ∂v_body / ∂δθ
        H.template block<3, 3>(0, 6) = Rt;                     // ∂v_body / ∂δv
        H.template block<3, 3>(3, 9) = Mat3T::Identity();       // ∂ω_body / ∂δω

        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = lin_vel_obs_body - v_body_hat;
        innov.template tail<3>() = ang_vel_obs_body - nominal.omega;

        return {H, innov};
    }
};

using NavigationModelF = NavigationModel<float>;
using NavigationModelD = NavigationModel<double>;

}  // namespace finenav
```

---

### 📄 finenav_localizer/include/finenav_localizer/finenav_localizer.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "eskf.hpp"
#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/nav_state_buffer.hpp"
#include "finenav_localizer/observation_queue.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_localizer/localizer_params.hpp"

namespace finenav {

// ============================================================================
//  FineNavLocalizer  —  timer-driven ESKF localizer with time-delay compensation
//
//  参数通过 generate_parameter_library 从 finenav_localizer.yaml 加载，
//  与框架其他组件（Navigator、NavStateManager）保持一致的配置风格。
//
//  Internal threading model
//  ─────────────────────────────────────────────────────────────────────────
//  A private sub-node (pnode_) runs inside a dedicated NodeThread.
//  timerCallback() is the sole writer of eskf_.
//
//  setInitialPose() may be called from any external thread; it uses
//  filter_mutex_ to serialise access with timerCallback().
//
//  feedObservation() pushes into pose_queue_ / twist_queue_, which are each
//  mutex-protected internally; the call never contends with timerCallback().
//
//  getState() / getCovariance() / getLastUpdateTime() read from an output
//  cache protected by cached_state_mutex_ (shared_mutex).
// ============================================================================
class FineNavLocalizer {
public:
    using PMat     = NavigationModelD::PMat;
    using Vec3d    = NavigationModelD::Vec3T;
    using SO3d     = NavigationModelD::SO3T;
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

    /// Convenience aliases for the generated parameter types.
    using Params        = finenav_localizer::Params;
    using ParamListener = finenav_localizer::ParamListener;

    /**
     * @brief 构造 FineNavLocalizer。
     * @param parent_node     宿主节点（用于获取 use_sim_time 及命名空间继承）。
     *                        内部会创建名为 "finenav_localizer" 的私有子节点来读取参数。
     * @param pnode_overrides 可选的参数覆盖（测试/eval场景用），通过
     *                        rclcpp::NodeOptions::append_parameter_override() 设置。
     */
    explicit FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                              rclcpp::NodeOptions pnode_overrides = rclcpp::NodeOptions{});

    FineNavLocalizer(const FineNavLocalizer &)             = delete;
    FineNavLocalizer & operator=(const FineNavLocalizer &) = delete;
    ~FineNavLocalizer() = default;

    /**
     * @brief Reset the filter to the given pose.  Clears all delay-slot history.
     * @note  Thread-safe.
     */
    void setInitialPose(const PoseMsg & msg);

    /**
     * @brief Enqueue a pose or twist observation for the next timer tick.
     * @note  Thread-safe.
     */
    void feedObservation(const PoseMsg  & msg);
    void feedObservation(const TwistMsg & msg);

    /**
     * @brief Read-only access to the latest filter output.
     *
     * @note  The returned NavStateD has `v` expressed in the **body frame**
     *        (R^T · v_world).  `p` and `R` are world-frame pose; `omega` is
     *        body-frame angular velocity (unchanged).
     * @note  Thread-safe.
     */
    NavStateD    getState()          const;
    PMat         getCovariance()     const;
    rclcpp::Time getLastUpdateTime() const;

    /**
     * @brief Query the interpolated state at an arbitrary past timestamp.
     *
     * Returns std::nullopt when the localizer is uninitialised, query_time is
     * in the future, or older than the retained history window.
     *
     * @note  `v` in the returned state is expressed in the **body frame**,
     *        consistent with getState().
     * @note  Thread-safe.
     */
    std::optional<NavStateD> getStateAt(const rclcpp::Time & query_time) const;

    /**
     * @brief Direct access to the underlying NavStateBuffer for advanced use-cases.
     * @note The buffer is always valid after construction.
     */
    NavStateBuffer::SharedPtr getStateBuffer() const { return state_buffer_; }

private:

    // ── Timer callback ───────────────────────────────────────────────────────
    void timerCallback();

    // ── Observation update (called inside timerCallback, single-threaded) ────
    void applyUpdate(const PoseMsg  & msg, int delay_step);
    void applyUpdate(const TwistMsg & msg, int delay_step);

    // ── Message extraction helpers ───────────────────────────────────────────
    static std::tuple<Vec3d, SO3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const PoseMsg & msg);

    static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const TwistMsg & msg);

    // ── ROS infrastructure ───────────────────────────────────────────────────
    rclcpp::Node::SharedPtr                        parent_node_;
    rclcpp::Node::SharedPtr                        pnode_;
    std::unique_ptr<NodeThread>                    pnode_thread_;
    rclcpp::TimerBase::SharedPtr                   update_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;  // null when publish_tf is false

    // ── Parameters ──────────────────────────────────────────────────────────
    std::unique_ptr<ParamListener> param_listener_;
    Params                         params_;

    // ── Filter state  (protected by filter_mutex_) ───────────────────────────
    mutable std::mutex          filter_mutex_;
    ESKFd                       eskf_;
    std::optional<rclcpp::Time> last_predict_time_;

    // ── Observation queues (each internally mutex-protected) ─────────────────
    std::unique_ptr<PoseObsQueue>  pose_queue_;
    std::unique_ptr<TwistObsQueue> twist_queue_;

    // ── Initialisation guard ─────────────────────────────────────────────────
    std::atomic<bool> is_initialized_{false};

    // ── Output cache  (protected by cached_state_mutex_) ────────────────────
    mutable std::shared_mutex cached_state_mutex_;
    NavStateD    cached_state_{};
    PMat         cached_covariance_ = PMat::Identity() * 1e-2;
    rclcpp::Time cached_time_{0, 0, RCL_ROS_TIME};

    // ── Stamped-history buffer (thread-safe internally) ──────────────────────
    NavStateBuffer::SharedPtr state_buffer_;

    rclcpp::Logger logger_;
};

}  // namespace finenav

```

---

### 📄 finenav_localizer/include/finenav_localizer/kalman_filter/kalman_filter.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>

namespace finenav {

// ============================================================================
//  KalmanFilter  —  standard linear discrete-time KF (dynamic matrices)
//
//  Prediction:  x' = x_next,   P' = A·P·Aᵀ + Q
//  Update:      K  = P·Cᵀ·(C·P·Cᵀ + R)⁻¹
//               x' = x + K·(y − y_pred)
//               P' = (I − K·C)·P·(I − K·C)ᵀ + K·R·Kᵀ   (Joseph form)
//
//  All matrices use double (Eigen::MatrixXd).  The class is header-only.
// ============================================================================
class KalmanFilter {
public:
    KalmanFilter()          = default;
    virtual ~KalmanFilter() = default;

    // ---- Initialization ---------------------------------------------------

    /// Minimal init: only x₀ and P₀ (A/B/C/Q/R must be set separately if needed).
    bool init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0)
    {
        if (x.size() == 0 || P0.size() == 0) { return false; }
        x_ = x;
        P_ = P0;
        return true;
    }

    // ---- Prediction step -------------------------------------------------

    /// @param x_next  Pre-computed next state  (caller's responsibility to
    ///                apply the non-linear model before calling predict).
    /// @param A       State-transition Jacobian  (dim_x × dim_x).
    /// @param Q       Process-noise covariance   (dim_x × dim_x).
    bool predict(const Eigen::MatrixXd & x_next,
                 const Eigen::MatrixXd & A,
                 const Eigen::MatrixXd & Q)
    {
        if (x_.rows() != x_next.rows() ||
            A.cols()  != P_.rows()     ||
            Q.rows()  != Q.cols()      ||
            A.rows()  != Q.cols()) {
            return false;
        }
        x_ = x_next;
        P_ = A * P_ * A.transpose() + Q;
        return true;
    }

    // ---- Update step ------------------------------------------------------

    /// Full update with explicit prediction y_pred (useful when h(x) is
    /// non-linear — caller linearises and passes y_pred = h(x̂)).
    /// Uses Joseph form for numerical stability.
    bool update(const Eigen::MatrixXd & y,
                const Eigen::MatrixXd & y_pred,
                const Eigen::MatrixXd & C,
                const Eigen::MatrixXd & R)
    {
        if (P_.cols() != C.cols() ||
            R.rows() != R.cols()  ||
            R.rows() != C.rows()  ||
            y.rows() != y_pred.rows() ||
            y.rows() != C.rows()) {
            return false;
        }

        const Eigen::MatrixXd PCT = P_ * C.transpose();
        const Eigen::MatrixXd K   = PCT * (R + C * PCT).inverse();

        if (K.array().isNaN().any() || K.array().isInf().any()) {
            return false;
        }

        x_ = x_ + K * (y - y_pred);

        // Joseph form: P = (I-KC)P(I-KC)' + KRK'
        const Eigen::MatrixXd I_KC =
            Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * C;
        P_ = I_KC * P_ * I_KC.transpose() + K * R * K.transpose();

        return true;
    }

    /// Simplified update: y_pred = C·x̂  (linear observation model).
    bool update(const Eigen::MatrixXd & y,
                const Eigen::MatrixXd & C,
                const Eigen::MatrixXd & R)
    {
        if (C.cols() != x_.rows()) { return false; }
        return update(y, C * x_, C, R);
    }

    // ---- Accessors --------------------------------------------------------
    [[nodiscard]] Eigen::MatrixXd getX() const { return x_; }
    [[nodiscard]] Eigen::MatrixXd getP() const { return P_; }
    [[nodiscard]] double getXelement(int i) const { return x_(i); }

protected:
    Eigen::MatrixXd x_;   ///< state vector  (dim_x × 1)
    Eigen::MatrixXd P_;   ///< covariance    (dim_x × dim_x)
};

}  // namespace finenav

```

---

### 📄 finenav_localizer/include/finenav_localizer/kalman_filter/time_delay_kalman_filter.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_localizer/kalman_filter/kalman_filter.hpp"

#include <iostream>
#include <utility>

namespace finenav {

// ============================================================================
//  TimeDelayKalmanFilter  —  KF with state-expansion for delayed observations
//
//  Extends KalmanFilter by maintaining an augmented state vector:
//
//      x_ex = [ x_k | x_{k-1} | ... | x_{k-d+1} ]   (dim_x * max_delay_step)
//
//  The expanded state transition matrix has the structure:
//
//      A_ex = [ A   0   0 … ]      Q_ex = [ Q   0   0 … ]
//             [ I   0   0 … ]             [ 0   0   0 … ]
//             [ 0   I   0 … ]             [ 0   0   0 … ]
//             [ …   …   … … ]             [ …   …   … … ]
//
//  An observation arriving with delay_step d is associated with x_{k-d} and
//  uses the measurement matrix:
//
//      C_ex = [ 0 | … | C | … | 0 ]   (C at column block d)
//
//  This lets the standard KF update simultaneously correct x_k (via the
//  cross-covariance blocks) and x_{k-d} (directly).
//
//  ── Usage ─────────────────────────────────────────────────────────────────
//  1. Call init(x0, P0, max_delay_step) once.
//  2. Every filter tick:  predictWithDelay(x_next, F, Q)
//  3. For each observation with known delay_step:
//       updateWithDelay(y, C, R, delay_step)
//  4. getLatestX() / getLatestP() return the leading (current-time) block.
//  5. After ESKF error-state reset, call resetState() to zero x_ex.
//  6. After ESKF covariance projection, call setLeadingCovariance(P_proj).
// ============================================================================
class TimeDelayKalmanFilter : public KalmanFilter {
public:
    TimeDelayKalmanFilter() = default;

    // ---- Initialization ---------------------------------------------------

    /// @param x              Initial state vector        (dim_x × 1)
    /// @param P0             Initial covariance           (dim_x × dim_x)
    /// @param max_delay_step Maximum number of delay slots (≥ 1)
    ///
    /// All delay slots are seeded with the same (x, P0) so that the filter
    /// is consistent from the first tick.
    void init(const Eigen::MatrixXd & x,
              const Eigen::MatrixXd & P0,
              int                    max_delay_step)
    {
        max_delay_step_ = max_delay_step;
        dim_x_          = static_cast<int>(x.rows());
        dim_x_ex_       = dim_x_ * max_delay_step_;

        // Fill all delay slots with the same initial state / covariance.
        x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        for (int i = 0; i < max_delay_step_; ++i) {
            x_.block(i * dim_x_, 0,          dim_x_, 1)      = x;
            P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
        }
    }

    // ---- Accessors for the leading (current-time) block ------------------
    [[nodiscard]] Eigen::MatrixXd getLatestX() const
    {
        return x_.block(0, 0, dim_x_, 1);
    }

    [[nodiscard]] Eigen::MatrixXd getLatestP() const
    {
        return P_.block(0, 0, dim_x_, dim_x_);
    }

    // ---- ESKF helpers ----------------------------------------------------

    /// Zero out the entire extended error-state vector.
    /// Called after ESKF applies the error correction to the nominal state.
    void resetState()
    {
        x_.setZero();
    }

    /// Overwrite the leading (0,0) covariance block only.
    /// Called after ESKF covariance projection (P ← G·P·Gᵀ).
    void setLeadingCovariance(const Eigen::MatrixXd & P_new)
    {
        P_.block(0, 0, dim_x_, dim_x_) = P_new;
    }

    /// Read the error-state vector at a specific delay slot.
    /// slot 0 = current time (same as getLatestX()), slot d = d ticks ago.
    [[nodiscard]] Eigen::MatrixXd getXAtSlot(int slot) const
    {
        return x_.block(slot * dim_x_, 0, dim_x_, 1);
    }

    // ---- Time-delay predict ----------------------------------------------

    /// Slide all slots one step forward in time, then update covariance.
    ///
    /// Extended state slide:
    ///   x_ex ← [ x_next | x_k | x_{k-1} | … | x_{k-d+2} ]
    ///           (oldest slot x_{k-d+1} is discarded)
    ///
    /// Extended covariance (structured propagation, see class comment):
    ///   P(0,0) ← A·P(0,0)·Aᵀ + Q
    ///   P(0,j) ← A·P(0,j-1)     for j ≥ 1
    ///   P(i,0) ← P(i-1,0)·Aᵀ   for i ≥ 1
    ///   P(i,j) ← P(i-1,j-1)     for i,j ≥ 1
    ///
    /// @param x_next  Next nominal error state (zero in ESKF, since error
    ///                state is always reset after every update).
    /// @param A       Error-state transition Jacobian F  (dim_x × dim_x).
    /// @param Q       Process-noise covariance           (dim_x × dim_x).
    bool predictWithDelay(const Eigen::MatrixXd & x_next,
                          const Eigen::MatrixXd & A,
                          const Eigen::MatrixXd & Q)
    {
        const int d_dim_x = dim_x_ex_ - dim_x_;  // (max_delay_step - 1) * dim_x

        // ── Slide state vector ──────────────────────────────────────────────
        // x_tmp = [x_next; x_[0 .. d_dim_x-1]]
        Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        x_tmp.block(0,      0, dim_x_,  1) = x_next;
        x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
        x_ = std::move(x_tmp);

        // ── Update extended covariance ─────────────────────────────────────
        //
        //  Symbolically (using block notation [i,j]):
        //    P_tmp[0,0] = A * P[0,0] * A' + Q
        //    P_tmp[0, 1..] = A * P[0, 0..d-2]    (first row of new cross-cov)
        //    P_tmp[1.., 0] = P[0..d-2, 0] * A'   (first col of new cross-cov)
        //    P_tmp[1.., 1..] = P[0..d-2, 0..d-2] (inner block = old leading sub)
        //
        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        P_tmp.block(0,      0,      dim_x_,  dim_x_)  =
            A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose() + Q;

        P_tmp.block(0,      dim_x_, dim_x_,  d_dim_x) =
            A * P_.block(0, 0, dim_x_, d_dim_x);

        P_tmp.block(dim_x_, 0,      d_dim_x, dim_x_)  =
            P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();

        P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) =
            P_.block(0, 0, d_dim_x, d_dim_x);

        P_ = std::move(P_tmp);
        return true;
    }

    // ---- Delayed measurement update ---------------------------------------

    /// Apply a measurement that was captured delay_step ticks ago.
    ///
    /// Internally builds an expanded measurement matrix:
    ///   C_ex = [ 0 | … | C | … | 0 ]   (C at column block delay_step)
    ///
    /// The standard KF update then simultaneously corrects:
    ///   • x_{k-delay_step}  directly (via C)
    ///   • x_k, x_{k-1}, …  indirectly (via cross-covariance blocks)
    ///
    /// @param y           Measurement vector        (dim_y × 1)
    /// @param C           Measurement Jacobian      (dim_y × dim_x)
    /// @param R           Measurement noise cov.    (dim_y × dim_y)
    /// @param delay_step  0 = no delay (current tick), 1 = one tick old, …
    bool updateWithDelay(const Eigen::MatrixXd & y,
                         const Eigen::MatrixXd & C,
                         const Eigen::MatrixXd & R,
                         int                    delay_step)
    {
        if (delay_step >= max_delay_step_) {
            std::cerr << "[TimeDelayKF] delay_step=" << delay_step
                      << " >= max_delay_step=" << max_delay_step_
                      << " — ignoring update.\n";
            return false;
        }

        const int dim_y = static_cast<int>(y.rows());

        // Build expanded C: C_ex[:, delay_step*dim_x : (delay_step+1)*dim_x] = C
        Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
        C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

        // y_pred = C_ex * x_ (note: in ESKF x_ is always ≈ 0)
        return update(y, C_ex * x_, C_ex, R);
    }

    // ---- Introspection ----------------------------------------------------
    [[nodiscard]] int maxDelayStep() const { return max_delay_step_; }
    [[nodiscard]] int dimX()         const { return dim_x_; }
    [[nodiscard]] int dimXEx()       const { return dim_x_ex_; }

private:
    int max_delay_step_ = 1;
    int dim_x_          = 0;
    int dim_x_ex_       = 0;
};

}  // namespace finenav

```

---

### 📄 finenav_localizer/include/finenav_localizer/nav_state_buffer.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include <rclcpp/time.hpp>

#include "finenav_localizer/filter_model.hpp"

namespace finenav {

// ============================================================================
//  NavStateBuffer  —  rolling time-stamped history of NavStateD
//
//  Maintains a sliding window of (timestamp, state) pairs ordered by time.
//  Supports:
//    • push()       — append latest state, prune entries older than the window.
//    • getStateAt() — return interpolated state at an arbitrary past timestamp.
//                     Returns std::nullopt for timestamps outside [oldest, newest].
//    • getLatest()  — return the newest entry.
//    • clear()      — discard all history (e.g. on re-initialisation).
//
//  Threading
//  ─────────────────────────────────────────────────────────────────────────
//  All public methods are thread-safe via an internal std::mutex.
//
//  Interpolation
//  ─────────────────────────────────────────────────────────────────────────
//  • p, v, omega : linear interpolation.
//  • R           : SO3 geodesic interpolation (Slerp via Sophus SO3::exp/log).
// ============================================================================
class NavStateBuffer {
public:
    using SharedPtr = std::shared_ptr<NavStateBuffer>;

    struct StampedState {
        rclcpp::Time stamp;
        NavStateD    state;
    };

    /**
     * @param history_duration_sec  Length of the rolling window [s].
     *                               Entries older than (newest_stamp - window) are pruned.
     */
    explicit NavStateBuffer(double history_duration_sec = 2.0)
        : history_duration_sec_(history_duration_sec)
    {}

    // ── Write ────────────────────────────────────────────────────────────────

    /// Append a new (stamp, state) pair and prune expired entries.
    /// @note stamp must be monotonically non-decreasing; out-of-order pushes are silently ignored.
    void push(const rclcpp::Time & stamp, const NavStateD & state)
    {
        std::lock_guard<std::mutex> lk(mutex_);

        // Reject out-of-order updates.
        if (!buffer_.empty() && stamp < buffer_.back().stamp) {
            return;
        }

        buffer_.push_back({stamp, state});

        // Prune entries that fall outside the history window.
        prune(stamp);
    }

    /// Clear all history (call after setInitialPose).
    void clear()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.clear();
    }

    // ── Read ─────────────────────────────────────────────────────────────────

    /**
     * @brief Interpolate (or clamp) the state at the given timestamp.
     *
     * Returns std::nullopt if:
     *   - The buffer is empty.
     *   - @p query_time is strictly newer than the latest entry (future queries refused).
     *   - @p query_time is strictly older than the oldest retained entry.
     *
     * If @p query_time exactly matches an entry, that entry is returned directly.
     */
    std::optional<NavStateD> getStateAt(const rclcpp::Time & query_time) const
    {
        std::lock_guard<std::mutex> lk(mutex_);

        if (buffer_.empty()) {
            return std::nullopt;
        }

        // Refuse future queries.
        if (query_time > buffer_.back().stamp) {
            return std::nullopt;
        }

        // Clamp: query older than retained window → no data.
        if (query_time < buffer_.front().stamp) {
            return std::nullopt;
        }

        // Binary search: find the first entry with stamp >= query_time.
        auto it = std::lower_bound(
            buffer_.begin(), buffer_.end(), query_time,
            [](const StampedState & s, const rclcpp::Time & t) {
                return s.stamp < t;
            });

        // Exact match.
        if (it->stamp == query_time) {
            return it->state;
        }

        // it points to the first entry AFTER query_time; interpolate between
        // (it-1) and it.
        const auto & s1 = *(it - 1);
        const auto & s2 = *it;

        const double span = (s2.stamp - s1.stamp).seconds();
        if (span <= 0.0) {
            return s1.state;  // degenerate: identical stamps
        }

        const double alpha = (query_time - s1.stamp).seconds() / span;
        return interpolate(s1.state, s2.state, alpha);
    }

    /// Return the newest (stamp, state) pair, or nullopt if buffer is empty.
    std::optional<StampedState> getLatest() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.empty()) {
            return std::nullopt;
        }
        return buffer_.back();
    }

    /// Number of retained entries (primarily for diagnostics / tests).
    std::size_t size() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return buffer_.size();
    }

private:
    // ── Helpers ──────────────────────────────────────────────────────────────

    /// Prune entries older than (newest_stamp - history_duration_sec_).
    /// Must be called with mutex_ held.
    void prune(const rclcpp::Time & newest)
    {
        if (history_duration_sec_ <= 0.0) {
            return;
        }
        // Compute cutoff as a duration subtraction to stay in RCL_ROS_TIME.
        const rclcpp::Duration window{
            static_cast<int32_t>(static_cast<int64_t>(history_duration_sec_) / 1),
            static_cast<uint32_t>(
                (history_duration_sec_ - static_cast<int64_t>(history_duration_sec_)) * 1e9)};
        const rclcpp::Time cutoff = newest - rclcpp::Duration::from_seconds(history_duration_sec_);

        while (buffer_.size() > 1 && buffer_.front().stamp < cutoff) {
            buffer_.pop_front();
        }
    }

    /// Linearly interpolate between two NavStateD at ratio alpha in [0,1].
    static NavStateD interpolate(const NavStateD & a, const NavStateD & b, double alpha)
    {
        NavStateD out;
        out.p     = a.p     + alpha * (b.p     - a.p);
        out.v     = a.v     + alpha * (b.v     - a.v);
        out.omega = a.omega + alpha * (b.omega - a.omega);

        // SO3 geodesic interpolation: a ⊕ (alpha · log(a⁻¹ · b))
        const Sophus::SO3d delta = a.R.inverse() * b.R;
        out.R = a.R * Sophus::SO3d::exp(alpha * delta.log());

        return out;
    }

    double                   history_duration_sec_;
    mutable std::mutex       mutex_;
    std::deque<StampedState> buffer_;
};

}  // namespace finenav

```

---

### 📄 finenav_localizer/include/finenav_localizer/observation_queue.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Based on autoware::ekf_localizer::AgedObjectQueue (Apache-2.0).

#pragma once

#include <cstddef>
#include <mutex>
#include <queue>

#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

namespace finenav {

// ============================================================================
//  AgedObservationQueue<MsgT>  —  smoothing-aware, mutex-protected FIFO
//
//  Based on autoware::ekf_localizer::AgedObjectQueue.
//
//  Smoothing mechanism
//  ─────────────────────────────────────────────────────────────────────────
//  Each push() enters the message with age = 0.
//  pop_increment_age() returns the front message and:
//    • if age + 1 < max_age → re-enqueues the message with age + 1
//    • if age + 1 >= max_age → discards (the entry has been processed max_age times)
//
//  Combined with scaling R_eff = R * smoothing_steps inside applyUpdate(),
//  this distributes a single observation over max_age filter ticks with a
//  proportionally inflated noise covariance, producing smooth output.
//
//  Caller pattern (inside timerCallback):
//
//      const std::size_t n = queue.size();       // capture before loop
//      for (std::size_t i = 0; i < n; ++i) {
//          auto msg = queue.pop_increment_age();  // may re-enqueue at back
//          // ... process msg ...
//      }
//
//  Thread safety
//  ─────────────────────────────────────────────────────────────────────────
//  All public methods are mutex-protected.  push() is safe to call from
//  external threads (e.g. ROS subscriber callbacks) while the timer callback
//  is draining the queue.
// ============================================================================
template <typename MsgT>
class AgedObservationQueue {
public:
    /// @param smoothing_steps  Number of times each observation is processed (≥ 1).
    ///                         Corresponds to AgedObjectQueue::max_age.
    /// @param max_queue_size   Hard capacity cap (prevents unbounded growth).
    explicit AgedObservationQueue(std::size_t smoothing_steps,
                                  std::size_t max_queue_size = 64)
        : max_age_       (smoothing_steps > 0u ? smoothing_steps : 1u)
        , max_queue_size_(max_queue_size > max_age_ ? max_queue_size : max_age_)
    {}

    // ── Push ─────────────────────────────────────────────────────────────────

    void push(const MsgT & msg)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (objects_.size() >= max_queue_size_) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("finenav.AgedObservationQueue"),
                "[AgedObservationQueue] capacity (%zu) exceeded — dropping oldest entry.",
                max_queue_size_);
            objects_.pop();
            ages_.pop();
        }
        objects_.push(msg);
        ages_.push(0u);
    }

    // ── Pop with aging ───────────────────────────────────────────────────────

    /// Remove and return the front entry.  If its resulting age < max_age_,
    /// re-enqueue a copy at the back so it is processed again next tick.
    MsgT pop_increment_age()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        const MsgT        msg = objects_.front();
        const std::size_t age = ages_.front() + 1u;
        objects_.pop();
        ages_.pop();
        if (age < max_age_) {
            objects_.push(msg);
            ages_.push(age);
        }
        return msg;
    }

    // ── Introspection ────────────────────────────────────────────────────────

    [[nodiscard]] std::size_t size() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return objects_.size();
    }

    [[nodiscard]] bool empty() const { return size() == 0u; }

    void clear()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        objects_ = std::queue<MsgT>{};
        ages_    = std::queue<std::size_t>{};
    }

    [[nodiscard]] std::size_t maxAge()       const { return max_age_; }
    [[nodiscard]] std::size_t maxQueueSize() const { return max_queue_size_; }

private:
    const std::size_t       max_age_;
    const std::size_t       max_queue_size_;
    mutable std::mutex      mutex_;
    std::queue<MsgT>        objects_;
    std::queue<std::size_t> ages_;
};

// ── Concrete type aliases ─────────────────────────────────────────────────────
using PoseObsQueue  = AgedObservationQueue<geometry_msgs::msg::PoseWithCovarianceStamped>;
using TwistObsQueue = AgedObservationQueue<geometry_msgs::msg::TwistWithCovarianceStamped>;

}  // namespace finenav
```

---

### 📄 finenav_localizer/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_localizer</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>rclcpp</depend>
    <depend>tf2_ros</depend>
    <depend>tf2_eigen</depend>

    <depend>eigen</depend>
    <depend>sophus</depend>
    <depend>fmt</depend>

    <depend>finenav_util</depend>
    <depend>generate_parameter_library</depend>

    <test_depend>ament_cmake_gtest</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_localizer/src/finenav_localizer.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/finenav_localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace finenav {

// ============================================================================
//  Constructor
// ============================================================================
FineNavLocalizer::FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                                   rclcpp::NodeOptions pnode_overrides)
    : parent_node_(std::move(parent_node))
    , logger_(rclcpp::get_logger("finenav.FineNavLocalizer"))
{
    // ── 创建私有子节点（参数前缀由 yaml 顶层 key "finenav_localizer" 决定）──
    pnode_overrides.append_parameter_override(
        "use_sim_time",
        parent_node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer",
        parent_node_->get_name(),
        pnode_overrides);

    // ── 读取参数 ──────────────────────────────────────────────────────────────
    param_listener_ = std::make_unique<ParamListener>(pnode_);
    params_ = param_listener_->get_params();

    // ── 初始化 ESKF ──────────────────────────────────────────────────────────
    NavigationModelD::Options model_opts;
    model_opts.vel_noise_std   = params_.model.vel_noise_std;
    model_opts.omega_noise_std = params_.model.omega_noise_std;

    const int max_delay_step = static_cast<int>(std::max(
        int64_t{1},
        static_cast<int64_t>(std::ceil(params_.max_delay_sec * params_.update_rate_hz))));
    eskf_ = ESKFd(model_opts, max_delay_step);

    pose_queue_  = std::make_unique<PoseObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.pose_smoothing_steps)),
        static_cast<std::size_t>(params_.max_obs_queue_size));
    twist_queue_ = std::make_unique<TwistObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.twist_smoothing_steps)),
        static_cast<std::size_t>(params_.max_obs_queue_size));

    // ── State history buffer ──────────────────────────────────────────────────
    state_buffer_ = std::make_shared<NavStateBuffer>(params_.history_duration_sec);

    // ── TF broadcaster (only if publish_tf is enabled) ───────────────────────
    if (params_.publish_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);
    }

    // ── 更新定时器 ────────────────────────────────────────────────────────────
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / params_.update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { timerCallback(); });

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Created. rate=%.1f Hz  max_delay=%.3f s  "
        "max_delay_step=%d  pose_smooth=%ld  twist_smooth=%ld",
        params_.update_rate_hz, params_.max_delay_sec,
        eskf_.maxDelayStep(),
        params_.pose_smoothing_steps, params_.twist_smoothing_steps);
}

// ============================================================================
//  setInitialPose  (thread-safe via filter_mutex_)
// ============================================================================
void FineNavLocalizer::setInitialPose(const PoseMsg & msg)
{
    auto [pos, rot, R_obs] = extractObservation(msg);

    NavStateD init_state;
    init_state.p     = pos;
    init_state.R     = rot;
    init_state.v     = Vec3d::Zero();
    init_state.omega = Vec3d::Zero();

    PMat P = PMat::Zero();
    P.block<3, 3>(0, 0) = R_obs.block<3, 3>(0, 0);              // position
    P.block<3, 3>(3, 3) = R_obs.block<3, 3>(3, 3);              // rotation
    P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 100.0;  // linear vel (unknown)
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 100.0;  // angular vel (unknown)

    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        // reinitialize() resets nominal state + ALL delay slots in TimeDelayKF
        // and resets accumulated_delay_times_ to 1e15.
        eskf_.reinitialize(init_state, P);
        pose_queue_->clear();
        twist_queue_->clear();
        last_predict_time_.reset();
    }

    {
        std::unique_lock<std::shared_mutex> lk(cached_state_mutex_);
        cached_state_      = init_state;
        cached_covariance_ = P;
        cached_time_       = rclcpp::Time(msg.header.stamp);
    }

    state_buffer_->clear();
    state_buffer_->push(rclcpp::Time(msg.header.stamp), init_state);

    is_initialized_.store(true, std::memory_order_release);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Initial pose set: p=[%.3f, %.3f, %.3f]",
        pos.x(), pos.y(), pos.z());
}

// ============================================================================
//  feedObservation  (thread-safe: queue has its own mutex)
// ============================================================================
void FineNavLocalizer::feedObservation(const PoseMsg & msg)
{
    pose_queue_->push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedObservation<Pose>  queued (queue=%zu)",
        pose_queue_->size());
}

void FineNavLocalizer::feedObservation(const TwistMsg & msg)
{
    twist_queue_->push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedObservation<Twist> queued (queue=%zu)",
        twist_queue_->size());
}

// ============================================================================
//  Getters  (shared_mutex — multiple readers allowed)
// ============================================================================
NavStateD FineNavLocalizer::getState() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    NavStateD s = cached_state_;
    // External API contract: v is expressed in the body frame.
    // Internally the ESKF propagates v in world frame; convert here.
    s.v = s.R.inverse() * s.v;
    return s;
}

FineNavLocalizer::PMat FineNavLocalizer::getCovariance() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_covariance_;
}

rclcpp::Time FineNavLocalizer::getLastUpdateTime() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_time_;
}

std::optional<NavStateD> FineNavLocalizer::getStateAt(const rclcpp::Time & query_time) const
{
    auto s = state_buffer_->getStateAt(query_time);
    if (s) {
        // Same contract as getState(): convert v from world → body frame.
        s->v = s->R.inverse() * s->v;
    }
    return s;
}

// ============================================================================
//  timerCallback  — the sole writer of eskf_
// ============================================================================
void FineNavLocalizer::timerCallback()
{
    if (!is_initialized_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            logger_, *pnode_->get_clock(), 2000,
            "[FineNavLocalizer] Not initialised — call setInitialPose() first.");
        return;
    }

    const rclcpp::Time now = pnode_->now();
    std::lock_guard<std::mutex> lk(filter_mutex_);

    // ── 1. Compute dt ──────────────────────────────────────────────────────
    double dt = 1.0 / params_.update_rate_hz;  // nominal fallback (first cycle)
    if (last_predict_time_.has_value()) {
        const double meas_dt = (now - *last_predict_time_).seconds();
        if (meas_dt < 0.0) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Time jumped backwards (dt=%.6f s) — skipping cycle.",
                meas_dt);
            last_predict_time_ = now;
            return;
        }
        if (meas_dt > params_.max_dt_sec) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Large dt (%.4f s > max %.4f s) — clamped.",
                meas_dt, params_.max_dt_sec);
            dt = params_.max_dt_sec;
        } else {
            dt = meas_dt;
        }
    }
    last_predict_time_ = now;

    // ── 2. Predict ─────────────────────────────────────────────────────────
    // eskf_.predict() internally propagates the nominal state, slides the
    // TimeDelayKF extended covariance, and updates accumulated_delay_times_.
    RCLCPP_DEBUG(logger_, "[FineNavLocalizer] predict  dt=%.5f s", dt);
    eskf_.predict(dt);

    // ── 3. Process pose observations ───────────────────────────────────────
    // Capture size before the loop: pop_increment_age() may re-enqueue at the
    // back, but we process only the items that were present at the start of
    // this tick.
    {
        const std::size_t n = pose_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] pose observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const PoseMsg pose = pose_queue_->pop_increment_age();

            double delay_time = (now - rclcpp::Time(pose.header.stamp)).seconds();

            // Tolerate small scheduler jitter: clamp near-zero negative delays.
            if (delay_time < -0.003) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Pose obs from future (delay=%.4f s) — skipping.",
                    delay_time);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Pose obs too old (delay=%.4f s, "
                    "max tracked=%.4f s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(pose, static_cast<int>(delay_step));
        }
    }

    // ── 4. Process twist observations ──────────────────────────────────────
    {
        const std::size_t n = twist_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] twist observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const TwistMsg twist = twist_queue_->pop_increment_age();

            double delay_time = (now - rclcpp::Time(twist.header.stamp)).seconds();

            if (delay_time < -0.003) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Twist obs from future (delay=%.4f s) — skipping.",
                    delay_time);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Twist obs too old (delay=%.4f s, "
                    "max tracked=%.4f s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(twist, static_cast<int>(delay_step));
        }
    }

    // ── 5. Update output cache ─────────────────────────────────────────────
    {
        std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
        cached_state_      = eskf_.getState();
        cached_covariance_ = eskf_.getCovariance();
        cached_time_       = now;
    }

    // ── 5b. Append to history buffer ──────────────────────────────────────
    state_buffer_->push(now, eskf_.getState());

    // ── 6. Broadcast TF: map → base_link ───────────────────────────────────
    const NavStateD & s = eskf_.getState();

    if (tf_br_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = "map";
        tf.child_frame_id  = "base_link";

        tf.transform.translation.x = s.p.x();
        tf.transform.translation.y = s.p.y();
        tf.transform.translation.z = s.p.z();

        const Eigen::Quaterniond q(s.R.matrix());
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        tf_br_->sendTransform(tf);
    }

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] tick  |p|=%.4f  |v|=%.4f  |ω|=%.4f",
        s.p.norm(), s.v.norm(), s.omega.norm());
}

// ============================================================================
//  applyUpdate  — Mahalanobis gate + ESKF update with delay compensation
// ============================================================================
void FineNavLocalizer::applyUpdate(const PoseMsg & msg, int delay_step)
{
    auto [pos_obs, rot_obs, R_raw] = extractObservation(msg);

    // Scale noise covariance by smoothing_steps: makes each of the N
    // processing steps less aggressive, resulting in a smooth correction
    // spread over N ticks.
    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(params_.pose_smoothing_steps);

    // Linearise at the nominal state from the time of capture, not the
    // current nominal.  This gives the correct innovation z − h(x̂_{k−d})
    // and the correct H Jacobian for the delayed KF update.
    const auto obs = NavigationModelD::observePose(
        eskf_.getStateAtDelay(delay_step), pos_obs, rot_obs);

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    // Gate uses the CURRENT covariance (leading block of extended P).
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovariance() * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    if (d2 > params_.pose_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<Pose> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.pose_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<Pose>   delay_step=%d  d²=%.4f  "
        "|δp|=%.4f  |δR|=%.4f",
        delay_step, d2,
        obs.innovation.head<3>().norm(), obs.innovation.tail<3>().norm());
}

void FineNavLocalizer::applyUpdate(const TwistMsg & msg, int delay_step)
{
    auto [lin_vel, ang_vel, R_raw] = extractObservation(msg);

    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(params_.twist_smoothing_steps);

    // Same as pose: linearise at the historical nominal from delay_step ticks ago.
    const auto obs = NavigationModelD::observeVelocity(
        eskf_.getStateAtDelay(delay_step), lin_vel, ang_vel);

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovariance() * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    if (d2 > params_.twist_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<Twist> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.twist_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<Twist>  delay_step=%d  d²=%.4f  "
        "|δv|=%.4f  |δω|=%.4f",
        delay_step, d2,
        obs.innovation.head<3>().norm(), obs.innovation.tail<3>().norm());
}

// ============================================================================
//  extractObservation  —  message → filter-ready quantities
// ============================================================================
std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::SO3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractObservation(const PoseMsg & msg)
{
    Vec3d pos{
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z};

    const Eigen::Quaterniond q{
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z};
    const SO3d rot{q.normalized().toRotationMatrix()};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.pose.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-9);

    return {pos, rot, R};
}

std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::Vec3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractObservation(const TwistMsg & msg)
{
    Vec3d lin{
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z};

    Vec3d ang{
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.twist.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-9);

    return {lin, ang, R};
}

}  // namespace finenav


```

---

### 📄 finenav_localizer/src/finenav_localizer.yaml

```yaml
finenav_localizer:
  update_rate_hz: {
    type: double,
    default_value: 100.0,
    description: "Timer frequency [Hz] for the ESKF update loop",
    validation: {
      bounds<>: [1.0, 1000.0]
    }
  }
  max_delay_sec: {
    type: double,
    default_value: 0.5,
    description: "Maximum tolerable observation delay [s]",
    validation: {
      bounds<>: [0.0, 5.0]
    }
  }
  pose_smoothing_steps: {
    type: int,
    default_value: 1,
    description: "Number of times a pose observation is reused; R is scaled by this factor",
    validation: {
      bounds<>: [1, 20]
    }
  }
  twist_smoothing_steps: {
    type: int,
    default_value: 1,
    description: "Number of times a twist observation is reused; R is scaled by this factor",
    validation: {
      bounds<>: [1, 20]
    }
  }
  pose_gate_dist: {
    type: double,
    default_value: 1000000000.0,
    description: "Mahalanobis gate distance for pose observations; set large (>=0) to disable gating",
    validation: {
      bounds<>: [0.0, 1000000000.0]
    }
  }
  twist_gate_dist: {
    type: double,
    default_value: 1000000000.0,
    description: "Mahalanobis gate distance for twist observations; set large (>=0) to disable gating",
    validation: {
      bounds<>: [0.0, 1000000000.0]
    }
  }
  max_dt_sec: {
    type: double,
    default_value: 0.5,
    description: "Upper bound clamp for dt [s]",
    validation: {
      bounds<>: [0.001, 5.0]
    }
  }
  max_obs_queue_size: {
    type: int,
    default_value: 64,
    description: "Per-type observation queue capacity",
    validation: {
      bounds<>: [1, 1024]
    }
  }
  history_duration_sec: {
    type: double,
    default_value: 2.0,
    description: "Duration [s] of the rolling NavStateBuffer history window for time-stamped state queries",
    validation: {
      bounds<>: [0.1, 60.0]
    }
  }
  publish_tf: {
    type: bool,
    default_value: false,
    description: "Whether to broadcast the map->base_link TF transform on each filter update tick",
  }
  model:
    vel_noise_std: {
      type: double,
      default_value: 0.10,
      description: "Linear velocity process noise std [m/s/√s]",
      validation: {
        bounds<>: [0.000001, 100.0]
      }
    }
    omega_noise_std: {
      type: double,
      default_value: 0.10,
      description: "Angular velocity process noise std [rad/s/√s]",
      validation: {
        bounds<>: [0.000001, 100.0]
      }
    }



```

---

### 📄 finenav_localizer/test/CMakeLists.txt

```
ament_add_gtest(${PROJECT_NAME}_test
        test_localizer_units.cpp
)
target_link_libraries(${PROJECT_NAME}_test
        finenav_localizer
)
```

---

### 📄 finenav_localizer/test/test_localizer_units.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <cmath>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/finenav_localizer.hpp"

using namespace std::chrono_literals;
namespace fn = finenav;
using Model = fn::NavigationModelD;

// ============================================================================
//  NavigationModel (pure math — no ROS required)
// ============================================================================

TEST(NavigationModel, PredictNominal_ZeroVelocity_StaysAtOrigin)
{
    fn::NavStateD s;  // default: p=0, R=I, v=0, ω=0
    const auto ns = Model::predictNominal(s, 1.0);
    EXPECT_NEAR(ns.p.norm(), 0.0, 1e-12);
    EXPECT_NEAR((ns.R.matrix() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(NavigationModel, PredictNominal_ConstantVelocity_PositionIntegrates)
{
    fn::NavStateD s;
    s.v = {1.0, 2.0, 0.0};
    const auto ns = Model::predictNominal(s, 3.0);
    EXPECT_NEAR(ns.p.x(), 3.0, 1e-12);
    EXPECT_NEAR(ns.p.y(), 6.0, 1e-12);
    EXPECT_NEAR(ns.p.z(), 0.0, 1e-12);
}

TEST(NavigationModel, ObservePose_ZeroInnovationAtNominal)
{
    fn::NavStateD s;
    s.p = {1.0, -2.0, 0.5};
    s.R = Sophus::SO3d::exp(Eigen::Vector3d{0.0, 0.0, 1.2});

    const auto obs = Model::observePose(s, s.p, s.R);
    EXPECT_NEAR(obs.innovation.norm(), 0.0, 1e-10);
}

TEST(NavigationModel, ObserveVelocity_ZeroInnovationAtNominal)
{
    fn::NavStateD s;
    s.v     = {1.0, 0.5, 0.0};
    s.omega = {0.0, 0.0, 0.3};

    // body-frame velocity matching the nominal
    const Eigen::Vector3d v_body = s.R.inverse() * s.v;
    const auto obs = Model::observeVelocity(s, v_body, s.omega);
    EXPECT_NEAR(obs.innovation.norm(), 0.0, 1e-9);
}

// ============================================================================
//  FineNavLocalizer integration (uses real wall-clock timer; stays < 500 ms)
// ============================================================================

class LocalizerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        node_ = rclcpp::Node::make_shared("localizer_unit_test");

        rclcpp::NodeOptions pnode_opts;
        pnode_opts.append_parameter_override("update_rate_hz",       100.0);
        pnode_opts.append_parameter_override("pose_smoothing_steps", int64_t{1});
        pnode_opts.append_parameter_override("pose_gate_dist",       20.0);
        pnode_opts.append_parameter_override("model.vel_noise_std",   0.1);
        pnode_opts.append_parameter_override("model.omega_noise_std", 0.1);

        localizer_ = std::make_unique<fn::FineNavLocalizer>(node_, pnode_opts);
    }

    /// Build a minimal PoseMsg at (x, y, yaw) with diagonal σ² covariance.
    static fn::FineNavLocalizer::PoseMsg makePose(
        double x, double y, double yaw, double pos_sigma,
        rclcpp::Time stamp)
    {
        fn::FineNavLocalizer::PoseMsg m;
        m.header.stamp    = stamp;
        m.header.frame_id = "map";
        m.pose.pose.position.x  = x;
        m.pose.pose.position.y  = y;
        m.pose.pose.position.z  = 0.0;
        m.pose.pose.orientation.w = std::cos(yaw * 0.5);
        m.pose.pose.orientation.x = 0.0;
        m.pose.pose.orientation.y = 0.0;
        m.pose.pose.orientation.z = std::sin(yaw * 0.5);
        m.pose.covariance.fill(0.0);
        const double var = pos_sigma * pos_sigma;
        m.pose.covariance[0]  = var;
        m.pose.covariance[7]  = var;
        m.pose.covariance[14] = 1e-4;
        m.pose.covariance[21] = 1e-4;
        m.pose.covariance[28] = 1e-4;
        m.pose.covariance[35] = var;
        return m;
    }

    rclcpp::Node::SharedPtr               node_;
    std::unique_ptr<fn::FineNavLocalizer> localizer_;
};

TEST_F(LocalizerTest, SetInitialPose_StateMatchesInputImmediately)
{
    constexpr double x = 3.0, y = -1.5, yaw = 0.8;
    localizer_->setInitialPose(makePose(x, y, yaw, 0.1, node_->now()));

    const fn::NavStateD s = localizer_->getState();
    EXPECT_NEAR(s.p.x(), x,   1e-9);
    EXPECT_NEAR(s.p.y(), y,   1e-9);

    const Eigen::Quaterniond q(s.R.matrix());
    const double yaw_out = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    EXPECT_NEAR(yaw_out, yaw, 1e-9);
}

TEST_F(LocalizerTest, FeedPoseObservation_StateMovesTowardObservation)
{
    // Initialise at origin with large uncertainty so the gate passes
    localizer_->setInitialPose(makePose(0.0, 0.0, 0.0, 1.0, node_->now()));
    rclcpp::sleep_for(20ms);

    // Feed a pose observation 1 m away
    constexpr double obs_x = 1.0, obs_y = 0.0;
    localizer_->feedObservation(makePose(obs_x, obs_y, 0.0, 0.1, node_->now()));

    rclcpp::sleep_for(150ms);  // let several timer ticks run

    const fn::NavStateD s = localizer_->getState();
    const double dist_init  = std::hypot(obs_x, obs_y);      // distance from 0,0 to obs
    const double dist_after = std::hypot(s.p.x() - obs_x, s.p.y() - obs_y);
    EXPECT_LT(dist_after, dist_init);  // state closer to obs than initial (0,0)
}

// ── Test entry point ──────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

```

---

### 📄 finenav_map/grid_map/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_map_grid_map)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(
        grid_map_params
        param/grid_map.yaml
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
        Eigen3::Eigen
        grid_map_params
)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_export_dependencies(
        eigen3_cmake_module
        Eigen3
)

ament_auto_package()
```

---

### 📄 finenav_map/grid_map/include/grid_map.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>

#include "finenav_map_grid_map/grid_map_params.hpp"
#include "finenav_util/algo_configurator.hpp"

#include "grid_map_math.hpp"
#include "grid_map_iterator.hpp"

namespace finenav { // TODO: 架构大改造，嵌套命名空间

/**
 * @brief 支持滚动更新的三维栅格地图类
 * @details 该地图维护两种索引方式，(1) Index 相对于地图原点的索引; (2) Position 相对于世界坐标系的坐标
 * @details 地图内部通过维护地图原点在世界坐标系中的位置，实现地图的滚动更新
 * @note 该栅格地图的原点总是在地图中心
 * @note 不支持地图原点的姿态变化
 */
template <typename T>
class GridMap {
public:
	using DataType = T;
    using ConfigType = grid_map::Params;

    // 1. 延迟初始化问题：构造之后不着急初始化，要处理 ROS 参数读取，再进行初始化配置
    // 2. 怎么读 ROS 参数，这是个问题: 弄个 ROSWrapper, 需要传递 node 进去，而后在合适时候调用 configure(node)，通过 generate_parameter_library
    // 读取预定义在 .yaml 中的参数，然后 GridMap 在内部处理这些 param 进行配置

	GridMap();

    /**
     * @brief 构造函数
     * @param length 栅格地图的尺寸，单位为米
     * @param resolution 栅格地图的分辨率，单位为米
     * @param origin 栅格地图的原点坐标
     * @note 所有地图数据均为零初始化
     */
    GridMap(const Length& length, const double& resolution, const Position& origin = Position::Zero());


    void configure(const ConfigType& params) {
        // TODO: 之后整，主要把 GridMap( Length, double, Poosition ) 的构造函数搬过来
        std::cout << "[GridMap] configure with params" << std::endl;

    // ===== 固定测试参数 =====
    resolution_ = 0.05;                     // 5cm
    length_ = Length(5.0, 5.0, 5.0);     // 10m x 10m x 3m
    origin_ = Position(0.0, 0.0, 0.0);

    // ===== 派生量 =====
    inv_resolution_ = 1.0 / resolution_;

    size_ = (length_.array() * inv_resolution_).ceil().cast<int>();

    // 保证是奇数（关键）
    size_ = size_.unaryExpr([](int v) {
        return (v % 2 == 0) ? v + 1 : v;
    });

    half_size_ = size_ / 2;

    // ===== 分配内存（核心）=====
    const size_t total_size =
        static_cast<size_t>(size_.x()) *
        static_cast<size_t>(size_.y()) *
        static_cast<size_t>(size_.z());

    data_.clear();
    data_.resize(total_size);

    // 初始化每个 voxel
    for (auto& cell : data_) {
        cell = T{};   // 你的 Voxel 默认是 NAN / false / 0
    }

    start_index_ = Index::Zero();

    // ===== debug 输出 =====
    std::cout << "[GridMap] resolution: " << resolution_ << std::endl;
    std::cout << "[GridMap] size: "
              << size_.x() << " x "
              << size_.y() << " x "
              << size_.z() << std::endl;
        std::cout << "[GridMap] total cells: " << total_size << std::endl;
    }

    /**
     * @brief 默认的构造函数
     */
    GridMap(const GridMap&) = default;
    GridMap& operator=(const GridMap&) = default;
    GridMap(GridMap&&) = default;
    GridMap& operator=(GridMap&&) = default;

    /**
     * @brief 析构函数
     */
    virtual ~GridMap() = default;

    /**
     * @brief 访问某个位置的栅格数据
     * @param position 世界坐标系下的位置
     * @return 对应位置的栅格数据
     * @throw
     */
    T& atPosition(const Position& position);

    /**
     * @brief 访问某个坐标点的栅格数据（常量版本）
     * @param position 世界坐标系下的位置
     * @return 对应位置的栅格数据
     */
    const T& atPosition(const Position& position) const;

    // TMP
    const DataType& access(const Position& position) const {
        return atPosition(position);
    }

    bool integrate(const Position& position, const DataType& data) {
        T& cell = atPosition(position);
        cell = data; // 简单覆盖更新 | 实际上，这里应该对应具体的更新逻辑，比如 raycast
        return true;
    }

    Region3D getWindowBounds() const {
        Position min_pos = getPosition(getMinIndex());
        Position max_pos = getPosition(-getMinIndex());
        return Region3D{min_pos, max_pos};
    }

    Position getWindowCenter() const {
        return origin_;
    }

    /**
     * @brief 访问某个index的栅格数据
     * @param index 栅格地图的索引
     * @return 对应的栅格数据
     */
    T& at(const Index& index);

    /**
     * @brief 访问某个index的栅格数据（常量版本）
     * @param index 栅格地图的索引
     * @return 对应的栅格数据
     */
    const T& at(const Index& index) const; // TODO：返回值类型改为 const T&


    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据
     * @param x x索引
     * @param y y索引
     * @return 深拷贝的栅格数据向量，正确处理滚动缓冲区的 z 轴环绕
     */
    std::vector<T> getVoxelsAlongZ(const int& x, const int& y);


    /**
     * @brief 访问某个(x,y)索引上的一串栅格数据（常量版本）
     * @param x x索引
     * @param y y索引
     * @return 深拷贝的栅格数据向量，正确处理滚动缓冲区的 z 轴环绕
     */
    std::vector<T> getVoxelsAlongZ(const int& x, const int& y) const;

    /**
     * @brief 清空地图数据
     */
    void clear();

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param position  地图原点移动到的新的位置
     * @return 如果地图实际发生移动返回true，否则返回false
     * @note 删除的栅格数据会被清除
     */
    bool moveTo(const Position& position);

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param[in] position 地图原点移动到的新的位置
     * @param[out] removed_region 移动过程中被删除的栅格数据及其对应的位置，仅当keep_removed为true时有效
     * @return 如果地图实际发生移动返回true，否则返回false
     */
    bool moveTo(const Position& position, std::vector<std::pair<Position, T>>& removed_region);

    /**
     * @brief 移动地图，这会造成栅格数据的更新
     * @param[in] position 地图原点移动到的新的位置
     * @param[in] keep_removed 是否保留被删除的栅格数据
     * @param[out] removed_region 移动过程中被删除的栅格数据及其对应的位置，仅当keep_removed为true时有效
     * @return 如果地图实际发生移动返回true，否则返回false
     */
    bool moveTo(const Position& position, const bool keep_removed, std::vector<std::pair<Position, T>>& removed_region);

    /**
     * @brief 将地图视窗平移到新的中心位置（MapConcept 要求）
     * @param position 新的视窗中心位置（世界坐标）
     */
    void shiftWindowTo(const Position& position);

    /**
     * @brief 光线投射
     * @param origin 光线的终点位置
     * @param end 光线的终点位置
     * @param indices 返回光线经过的栅格索引
     * @return 如果光线终点超出地图范围，返回false；否则返回true
     */
    bool rayCast(const Position& origin,const Position& end, std::vector<Index>& indices) const;

    /**
     * @brief 设置地图的原点，即地图在世界坐标系中的位置
     * @param origin 地图的原点坐标
     */
    void setOrigin(const Position& origin);

    /**
     * @brief 获取满足某个条件的栅格索引
     * @param[out] indices 输出满足条件的栅格索引
     * @param[in] condition 条件函数，接受栅格数据类型T的引用，返回bool值
     */
    void selectCellsByCondition(std::vector<Index>& indices, std::function<bool(const T&)> condition) const;

    /**
     * @brief 获取地图中某个位置的索引
     * @param position 世界坐标系下的位置
     * @return 对应的栅格地图索引
     */
    Index getIndex(const Position& position) const;

    /**
     * @brief 获取地图中某个索引对应的栅格中心位置
     * @param index 栅格地图的索引
     * @return 对应的栅格中心位置
     */
    Position getPosition(const Index& index) const;

    /**
     * @brief 获取地图的尺寸
     * @return 地图的尺寸，单位为米
     */
    Length getLength() const;

    /**
     * @brief 获取栅格地图的大小
     * @return 栅格地图的大小，单位为栅格数
     */
    Size getSize() const;

    /**
     * @brief 获取地图分辨率
     * @return 地图分辨率，单位为米
     */
    double getResolution() const;

    /**
     * @brief 获取地图原点
     * @return 地图原点坐标
     */
    Position getOrigin() const;

    /**
     * @brief 地图边界框的最小索引
     * @return min_index
     */
    Index getMinIndex() const;


    /**
     * @brief 地图边界框的最大索引
     * @return max_index
     */
    Index getMaxIndex() const;

    /**
     * @brief 检查某个位置是否在地图范围内
     * @param position 世界坐标系下的位置
     * @return 如果位置在地图范围内返回true，否则返回false
     */
    bool isInside(const Position& position) const;

    /**
     * @brief 检查某个位置是否在地图范围内
     * @param index 栅格地图的索引
     * @return 如果位置在地图范围内返回true，否则返回false
     */
    bool isInside(const Index& index) const;


    /// @brief 迭代器接口
    using iterator = GridMapIterator<T>;
    using const_iterator = GridMapIterator<const T>;

    iterator begin() { return iterator(this, false); }
    iterator end() { return iterator(this, true); }
    const_iterator begin() const { return const_iterator(this, false); }
    const_iterator end() const { return const_iterator(this, true); }

private:
    std::vector<T> data_; // 存储所有栅格数据
    Length length_;       // 栅格地图的长度，单位为米
    Index start_index_;   // 地图数据的起始索引
    Size size_;           // 栅格地图的尺寸，一定为奇数，以保证原点在中心位置
    double resolution_;   // 栅格地图的分辨率，单位为米
    Position origin_;     // 当前栅格地图的原点，用于定义世界坐标系下地图的位置

    double inv_resolution_;
    Size half_size_;

};

}

#include "grid_map_impl.hpp"

FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG(finenav::GridMap, grid_map)
```

---

### 📄 finenav_map/grid_map/include/grid_map_impl.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <limits>
#include <algorithm>
#include "grid_map.hpp"
#include <cmath>


namespace finenav {

template <typename T>
GridMap<T>::GridMap()
    : length_(Length::Zero()), start_index_(Index::Zero()), size_(Size::Zero()),
      resolution_(1.0), origin_(Position::Zero()), inv_resolution_(1.0), half_size_(Size::Zero()) {
    data_.clear();
}

template <typename T>
GridMap<T>::GridMap(const Length& length, const double& resolution, const Position& origin)
    : length_(length), resolution_(resolution), origin_(origin) {

    inv_resolution_ = 1.0 / resolution;

    size_ = (length.array() * inv_resolution_).ceil().cast<int>();
    size_ = size_.unaryExpr([](int v) {
        return (v % 2 == 0) ? v + 1 : v; // 确保尺寸为奇数
    });

    half_size_ = size_ / 2;

    data_.resize(size_.x() * size_.y() * size_.z(), NAN);
    start_index_ = Index::Zero();
}

template <typename T>
T& GridMap<T>::atPosition(const Position& position) {
    return at(getIndex(position));
}

template <typename T>
const T& GridMap<T>::atPosition(const Position& position) const {
    return at(getIndex(position));
}

template <typename T>
T& GridMap<T>::at(const Index& index) {
    if (checkIfIndexValid(index, size_, half_size_)) {
        return data_[getBufferIndex(index, size_, half_size_, start_index_)];
    }
    throw std::out_of_range("Accessing grid out of bounds");
}

template <typename T>
const T& GridMap<T>::at(const Index& index) const {
    if (checkIfIndexValid(index, size_, half_size_)) {
        return data_[getBufferIndex(index, size_, half_size_, start_index_)];
    }
    throw std::out_of_range("Accessing grid out of bounds");
}

template <typename T>
std::vector<T> GridMap<T>::getVoxelsAlongZ(const int& x, const int& y) {
    if (std::abs(x) > half_size_.x() || std::abs(y) > half_size_.y()) {
        throw std::out_of_range("Accessing grid out of bounds");
    }
    // Deep-copy each z-layer individually via at(), which correctly handles
    // rolling-buffer wrap-around along z. A raw span over [buffer_start, +size_z)
    // is unsafe when start_index_.z() != 0 because the z-block can spill past
    // the end of data_.
    std::vector<T> result;
    result.reserve(size_.z());
    for (int z = -half_size_.z(); z <= half_size_.z(); ++z) {
        result.push_back(at(Index(x, y, z)));
    }
    return result;
}

template <typename T>
std::vector<T> GridMap<T>::getVoxelsAlongZ(const int& x, const int& y) const {
    if (std::abs(x) > half_size_.x() || std::abs(y) > half_size_.y()) {
        throw std::out_of_range("Accessing grid out of bounds");
    }
    std::vector<T> result;
    result.reserve(size_.z());
    for (int z = -half_size_.z(); z <= half_size_.z(); ++z) {
        result.push_back(at(Index(x, y, z)));
    }
    return result;
}

template <typename T>
void GridMap<T>::clear() {
    data_.clear();
    data_.resize(size_.x() * size_.y() * size_.z(), T{});
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position) {
    std::vector<std::pair<Position, T>> removed_region;
    return moveTo(position, false, removed_region);
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position, std::vector<std::pair<Position, T>>& removed_region) {
    return moveTo(position, false, removed_region);
}

template <typename T>
bool GridMap<T>::moveTo(const Position& position, const bool keep_removed, std::vector<std::pair<Position, T>>& removed_region) {
    removed_region.clear();

    // 计算移动了的栅格数
    const auto index_shift = getIndexShiftFromPositionShift(position - origin_, resolution_, inv_resolution_);
    if (index_shift.isZero()) {
        return false; // 没有移动
    }

    // 以移动后的地图作为固定坐标系，获取受移动影响的栅格
    std::vector<Index> indices;
    getDifferenceSet(index_shift, size_, half_size_, indices);

    for (const auto& index : indices) {
        removed_region.emplace_back(getPosition(index), at(index));
        if (!keep_removed) {
            if (checkIfIndexValid(index, size_, half_size_)) {
                // 对于通用 DataType T（例如自定义 Voxel），使用默认构造来清空数据
                data_[getBufferIndex(index, size_, half_size_, start_index_)] = T{}; // 清空数据
            }
        }
    }

    // 更新移动后的地图状态
    const auto aligned_position_shift = getPositionShiftFromIndexShift(index_shift, resolution_, inv_resolution_);
    origin_ += aligned_position_shift;
    start_index_ = wrapIndexToRange(start_index_ + index_shift, size_); // 地图移动时，逻辑原点在缓冲区中的位置同向移动

    return true;
}

template <typename T>
void GridMap<T>::shiftWindowTo(const Position& position) {
    // 视窗中心等价于地图原点，这里直接复用已有的滚动更新逻辑
    moveTo(position);
}

template <typename T>
bool GridMap<T>::rayCast(const Position& origin,const Position& end, std::vector<Index>& indices) const {

    indices.clear();

    double min_length = 0.5;
    double max_length = 5.0;
    auto origin_voxel = getIndex(origin);
    auto last_voxel = getIndex(end);
    auto current_voxel = origin_voxel;
//    // 光线超出地图范围，结束
//    if (!checkIfIndexValid(last_voxel, size_, half_size_)) {
//        return false;
//    }

    // 光线终点为原点，结束
    //   if (origin_index == last_index) {
    //       return true;
    //   }

    // TODO:要不要先把原点放进来

    // 初始化
    Vector direction (end - origin);
    Vector inv_direction = direction.cwiseInverse();


    auto length = direction.norm();
    if (length < min_length) {
        return false;
    }
//    if (length > max_length) {
//        return false;
//    }
    //   direction = direction/length;   对方向向量归一化会引入无理数可能导致浮点数精度造成误差

    Eigen::Vector3i tMult = {0,0,0};
    Vector tMax, tDelta;
    Vector tMax_start, tDelta_start;
    Eigen::Vector3i step;

    for (int i = 0; i < 3; ++i) {
        step[i] = sign(direction[i]);

        if (step[i] != 0) {
            double voxelBorder;
            if (step[i] > 0) {
                voxelBorder = (current_voxel[i] + 1) * resolution_ + getOrigin()[i];
            } else {
                voxelBorder = current_voxel[i] * resolution_ + getOrigin()[i];
            }

            // 计算到下一个边界的参数距离
            tMax_start[i] = (voxelBorder - origin[i]) * inv_direction[i];
			tMax[i] = tMax_start[i];
            // 计算穿越一个完整体素的参数距离
            tDelta_start[i] = resolution_ * std::abs(inv_direction[i]);
            tDelta[i] = tDelta_start[i];
        }
        else{
            tMax_start[i] = std::numeric_limits<double>::infinity();
            tMax[i] = std::numeric_limits<double>::infinity();
            tDelta_start[i] = std::numeric_limits<double>::infinity();
            tDelta[i] = std::numeric_limits<double>::infinity();
        }
    }

    // TODO:需不需要对step为负值的情况做处理
    indices.push_back(current_voxel);
    while(current_voxel != last_voxel) {
        if (!checkIfIndexValid(current_voxel, size_, half_size_)) {
            return false;
        }

        if (tMax.x() < tMax.y()) {
            if (tMax.x() < tMax.z()) {   // x最小
                current_voxel.x() += step.x();
                tMax.x() += tDelta.x();
            } else {    // z最小
                current_voxel.z() += step.z();
                tMax.z() += tDelta.z();
            }
        } else {
            if (tMax.y() < tMax.z()) { // y最小
                current_voxel.y() += step.y();
                tMax.y() += tDelta.y();
            } else { // z最小
                current_voxel.z() += step.z();
                tMax.z() += tDelta.z();
            }
        }

        indices.push_back(current_voxel);
    }


    // 光线投射算法实现
    // 这里可以使用Bresenham算法或其他光线投射算法
    // 需要根据end位置计算出经过的栅格索引，并存储在indices中
    // 返回true表示成功，false表示失败（例如光线超出地图范围）
    return true; // 需要实现具体逻辑
}


template <typename T>
void GridMap<T>::setOrigin(const Position& origin) {
    origin_ = origin;
}

template <typename T>
void GridMap<T>::selectCellsByCondition(std::vector<Index>& indices, std::function<bool(const T&)> condition) const {
    indices.clear();
    for (int x = -half_size_.x(); x <= half_size_.x(); ++x) {
        for (int y = -half_size_.y(); y <= half_size_.y(); ++y) {
            for (int z = -half_size_.z(); z <= half_size_.z(); ++z) {
                Index idx(x, y, z);
                if (condition(at(idx))) {
                    indices.push_back(idx);
                }
            }
        }
    }
}

template <typename T>
Index GridMap<T>::getIndex(const Position& position) const {
    return getIndexFromPosition(position, resolution_, inv_resolution_, origin_);
}

template <typename T>
Position GridMap<T>::getPosition(const Index& index) const {
    return getPositionFromIndex(index, resolution_, inv_resolution_, origin_);
}

template <typename T>
Length GridMap<T>::getLength() const {
    return length_;
}

template <typename T>
Size GridMap<T>::getSize() const {
    return size_;
}

template <typename T>
double GridMap<T>::getResolution() const {
    return resolution_;
}

template <typename T>
Position GridMap<T>::getOrigin() const {
    return origin_;
}

template <typename T>
Index GridMap<T>::getMinIndex() const{
    return -half_size_;
}

template <typename T>
Index GridMap<T>::getMaxIndex() const{
    return half_size_;
}

template <typename T>
bool GridMap<T>::isInside(const Position& position) const {
    return checkIfIndexValid(getIndex(position), size_, half_size_);
}

template <typename T>
bool GridMap<T>::isInside(const Index& index) const {
    return checkIfIndexValid(index, size_, half_size_);
}

}

```

---

### 📄 finenav_map/grid_map/include/grid_map_iterator.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <stdexcept>
#include "grid_map_math.hpp"

namespace finenav {

template <typename T>
class GridMap; // 前向声明

template <typename T>
class GridMapIterator {
public:

    using value_type = T;
    using pointer = T*;
    using reference = T&;
    using map_type = GridMap<T>;

    // 仅用于创建end迭代器
    GridMapIterator() : map_(nullptr), current_index_(Index::Zero()), half_size_(Size::Zero()), is_end_(true) {}

    // 初始化迭代器
    GridMapIterator(map_type* map, bool is_end = false) : map_(map), is_end_(is_end) {
        if (map_) {
            half_size_ = map_->getSize() / 2;
            if (!is_end) {
                // 开始迭代器从最小索引开始
                current_index_ = Index(-half_size_.x(), -half_size_.y(), -half_size_.z());
            } else {
                // 结束迭代器设置为超出范围的索引
                current_index_ = Index(half_size_.x() + 1, -half_size_.y(), -half_size_.z());
            }
        } else {
            current_index_ = Index::Zero();
            half_size_ = Size::Zero();
        }
    }

    // 解引用操作符
    reference operator*() const {
        if (!map_ || is_end_) {
            throw std::runtime_error("Dereferencing invalid iterator");
        }
        return map_->at(current_index_);
    }

    // 箭头操作符
    pointer operator->() const {
        if (!map_ || is_end_) {
            throw std::runtime_error("Dereferencing invalid iterator");
        }
        return &(map_->at(current_index_));
    }

    // 前缀递增操作符
    GridMapIterator& operator++() {
        if (!map_ || is_end_) {
            return *this;
        }

        // 按照z-y-x的顺序递增
        ++current_index_.z();
        if (current_index_.z() > half_size_.z()) {
            current_index_.z() = -half_size_.z();
            ++current_index_.y();
            if (current_index_.y() > half_size_.y()) {
                current_index_.y() = -half_size_.y();
                ++current_index_.x();
                if (current_index_.x() > half_size_.x()) {
                    // 到达末尾
                    is_end_ = true;
                }
            }
        }
        return *this;
    }

    // 后缀递增操作符
    GridMapIterator operator++(int) {
        GridMapIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    // 相等比较操作符
    bool operator==(const GridMapIterator& other) const {
        if (map_ != other.map_) return false;
        if (is_end_ && other.is_end_) return true;
        if (is_end_ != other.is_end_) return false;
        return current_index_ == other.current_index_;
    }

    // 不等比较操作符
    bool operator!=(const GridMapIterator& other) const {
        return !(*this == other);
    }

    // 获取当前索引
    const Index& getIndex() const {
        return current_index_;
    }

    // 获取当前位置（世界坐标）
    Position getPosition() const {
        if (!map_ || is_end_) {
            throw std::runtime_error("Getting position from invalid iterator");
        }
        return map_->getPosition(current_index_);
    }

protected:
    map_type* map_;
    Size half_size_;
    Index current_index_;
    bool is_end_;
};

}
```

---

### 📄 finenav_map/grid_map/include/grid_map_math.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace finenav {

using Vector3D   = Eigen::Vector3d;
using Position3D = Eigen::Vector3d;
using Orientation = Eigen::Quaterniond;
using Region3D = Eigen::AlignedBox3d;
using Pose = Eigen::Isometry3d;


// TMP
using Vector   = Eigen::Vector3d;
using Position = Eigen::Vector3d;
using Index = Eigen::Vector3i;
using Size = Eigen::Vector3i;
using Length = Eigen::Vector3d;
/**
 * @brief 将索引向量的每个元素取模，确保索引在指定的范围内
 * @param index 输入的索引
 * @param size 指定的范围大小
 * @return 取模后的索引向量
 * @note 输入的索引向量可以是负数元素
 * TODO: 是否能够优化expensive的取模运算
 */
inline int wrapIndexToRange(const int& index, const int& size) {
    // 通过分支减少高昂的取模运算
    if (index < size) {
        if (index >= 0) { // [0, size)
            return index;
        } else if (index >= -size) { // [-size, 0)
            return index + size;
        } else { // [-inf, -size)
            return index % size + size;
        }
    } else if (index < size*2) { // [size, 2*size)
        return index - size;
    } else { // [2*size, inf)
        return index % size;
    }
}

inline Index wrapIndexToRange(const Index& index, const Size& size) {
    Index result;
    for (int i = 0; i < index.size(); i++) {
        result[i] = wrapIndexToRange(index[i], size[i]);
    }
    return result;
}

/**
 * @brief 返回一个数的符号
 * @Tparam 数据类型，支持 >, < 操作符
 * @param x 输入的数值
 * @return -1 如果 x < 0, -1 如果 x == 0, 0 如果 x > 0, 1
 */
template <typename T>
inline int sign(T x) {
    return (x > 0) - (x < 0);
}


/**
 * @brief 将位置偏移转换为索引偏移
 * @param position_shift 位置偏移量
 * @param resolution 栅格地图的分辨率
 * @param inv_resolution 栅格地图的分辨率的倒数
 * @return 对应的索引偏移量
 */
inline Index getIndexShiftFromPositionShift(const Position& position_shift, const double& resolution, const double& inv_resolution) {
    Vector index_shift_vector = position_shift * inv_resolution;
    return index_shift_vector.array().round().matrix().cast<int>();

}

/**
 * @brief 将索引偏移转换为位置偏移
 * @param index_shift 索引偏移量
 * @param resolution 栅格地图的分辨率
 * @param inv_resolution 栅格地图的分辨率的倒数
 * @return 对应的位移偏移量
 */
inline Position getPositionShiftFromIndexShift(const Index& index_shift, const double& resolution, const double& inv_resolution) {
    return index_shift.cast<double>() * resolution;
}


/**
 * @brief 将世界坐标系下的位置转换为栅格地图索引
 * @param position 访问的世界坐标系下的位置
 * @param resolution 栅格地图的分辨率
 * @param inv_resolution 栅格地图的分辨率的倒数
 * @param origin 栅格地图的原点位置
 * @return 对应的栅格地图索引
 */
inline Index getIndexFromPosition(const Position& position, const double& resolution, const double& inv_resolution,
    const Position& origin) {
    return getIndexShiftFromPositionShift(position - origin, resolution, inv_resolution);
}

/**
 * @brief 将栅格地图索引转换为世界坐标系下的位置
 * @param index 栅格地图索引
 * @param resolution 栅格地图的分辨率
 * @param inv_resolution 栅格地图的分辨率的倒数
 * @param origin 栅格地图的原点位置
 * @return 对应的栅格中心在世界坐标系下的位置
 */
inline Position getPositionFromIndex(const Index& index, const double& resolution, const double& inv_resolution, const Position& origin) {
    return origin + getPositionShiftFromIndexShift(index, resolution, inv_resolution);
}

/**
 * @brief 将地图索引转换为缓冲区索引
 * @param index 地图索引
 * @param size 地图的大小
 * @param half_size 地图大小的一半
 * @param buffer_start_index 循环缓冲区的起始索引
 * @return 对应的缓冲区索引
 * @note 按照z-y-x的顺序在缓冲区存储
 */
inline int getBufferIndex(const Index& index,
                            const Size& size,
                            const Size& half_size,
                            const Index& buffer_start_index = Index::Zero()) {
    const Index unwrapped_index = half_size + index;
    const Index wrapped_index = wrapIndexToRange(unwrapped_index + buffer_start_index, size);
    return wrapped_index.z() + wrapped_index.y() * size.z() + wrapped_index.x() * size.y() * size.z();
}

/**
 * @brief 检查索引是否在栅格地图的有效范围内
 * @param index 栅格地图索引
 * @param size 栅格地图的大小
 * @param half_size 栅格地图大小的一半
 * @return 如果索引在范围内返回true，否则返回false
 */
inline bool checkIfIndexValid(const Index& index, const Size& size, const Size& half_size) {
    return (index.cwiseAbs().array() <= half_size.array()).all();
}

/**
 * @brief 对于两个栅格地图A和B，计算差集A-B
 * @param[in] index_shift 相对于地图A的索引偏移量
 * @param[in] size 两个栅格地图的大小，地图大小需要为奇数
 * @param [in] half_size 栅格地图大小的一半
 * @param[out] difference_indices 输出的差集索引
 * @note 索引定义在地图A的坐标系下，栅格地图的原点位于地图中心
 */
inline void getDifferenceSet(const Index& index_shift, const Size& size, const Size& half_size, std::vector<Index>& difference_indices) {
    difference_indices.clear();

    Index a_min = -half_size;
    Index a_max = half_size;

    Index b_min = -half_size + index_shift;
    Index b_max = half_size + index_shift;
    
    // 如果B完全在A外面，直接返回整个A
    if ((b_min.array() > half_size.array()).any() || (b_max.array() < -half_size.array()).any()) {
        for (int x = -half_size.x(); x <= half_size.x(); ++x) {
            for (int y = -half_size.y(); y <= half_size.y(); ++y) {
                for (int z = -half_size.z(); z <= half_size.z(); ++z) {
                    difference_indices.emplace_back(x, y, z);
                }
            }
        }
        return;
    }

    // 辅助函数：添加指定范围的索引到差集
    auto add_range = [&](int x_start, int x_end, int y_start, int y_end, int z_start, int z_end) {
        // 限制范围在 A 内
        x_start = std::max(x_start, a_min.x());
        x_end = std::min(x_end, a_max.x());
        y_start = std::max(y_start, a_min.y());
        y_end = std::min(y_end, a_max.y());
        z_start = std::max(z_start, a_min.z());
        z_end = std::min(z_end, a_max.z());

        if (x_start > x_end || y_start > y_end || z_start > z_end) return;

        for (int x = x_start; x <= x_end; ++x) {
            for (int y = y_start; y <= y_end; ++y) {
                for (int z = z_start; z <= z_end; ++z) {
                    difference_indices.emplace_back(x, y, z);
                }
            }
        }
    };
    // 依次判断 z,x,y
    if (a_min.z() <= b_max.z() && b_max.z() <= a_max.z()) {
        add_range(a_min.x(), a_max.x(), a_min.y(), a_max.y(), b_max.z() + 1, a_max.z());

        // x
        if (a_min.x() <= b_min.x() && b_min.x() <= a_max.x()) {
            add_range(a_min.x(), b_min.x()-1, a_min.y(), a_max.y(), a_min.z(), b_max.z());
            
            // y
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, a_min.z(), b_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), a_min.z(), b_max.z());
            }
        } else if(a_min.x() <= b_max.x() && b_max.x() <= a_max.x()){
            add_range(b_max.x()+1, a_max.x(), a_min.y(), a_max.y(), a_min.z(), b_max.z());
            
            // y
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, a_min.z(), b_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), a_min.z(), b_max.z());
            }
        }
    } else if(a_min.z() <= b_min.z() && b_min.z() <= a_max.z()){
        add_range(a_min.x(), a_max.x(), a_min.y(), a_max.y(), a_min.z(), b_min.z() - 1);

        // x
        if (a_min.x() <= b_min.x() && b_min.x() <= a_max.x()) {
            add_range(a_min.x(), b_min.x()-1, a_min.y(), a_max.y(), b_min.z(), a_max.z());
            
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, b_min.z(), a_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), b_min.z(), a_max.z());
            }
        } else if (a_min.x() <= b_max.x() && b_max.x() <= a_max.x()){
            add_range(b_max.x()+1, a_max.x(), a_min.y(), a_max.y(), b_min.z(), a_max.z());
            
            if (a_min.y() <= b_min.y() && b_min.y() <= a_max.y()) {
                add_range(b_min.x(), b_max.x(), a_min.y(), b_min.y() - 1, b_min.z(), a_max.z());
            } else if(a_min.y() <= b_max.y() && b_max.y() <= a_max.y()){
                add_range(b_min.x(), b_max.x(), b_max.y() + 1, a_max.y(), b_min.z(), a_max.z());
            }
        }
    }
}

}
```

---

### 📄 finenav_map/grid_map/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_map_grid_map</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake_auto</buildtool_depend>

    <build_depend>eigen3_cmake_module</build_depend>
    <exec_depend>eigen3_cmake_module</exec_depend>
    <build_export_depend>eigen3_cmake_module</build_export_depend>

    <depend>generate_parameter_library</depend>
    <depend>finenav_core</depend>
    <depend>finenav_util</depend>

    <test_depend>ament_cmake_gtest</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_map/grid_map/param/grid_map.yaml

```yaml
grid_map:
  background:
    r:
      type: int
      default_value: 0
      description: "Red color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    g:
      type: int
      default_value: 0
      description: "Green color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    b:
      type: int
      default_value: 0
      description: "Blue color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
```

---

### 📄 finenav_motion_decorator/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_motion_decorator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/trim_by_count.cpp
    src/trim_by_duration.cpp
    src/trim_by_distance.cpp
    src/seek_to_nearest_point.cpp
    src/trim_by_aabb.cpp
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package()
```

---

### 📄 finenav_motion_decorator/include/detail/trim_utils.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_msgs/msg/trajectory.hpp"
#include <cstddef>

namespace finenav::motion_decorator::detail {

/**
 * @brief Resize all populated fields of a trajectory to new_size consistently.
 *        Caller is responsible for ensuring new_size <= current size.
 */
inline void resize_trajectory(finenav_msgs::msg::Trajectory& traj, std::size_t new_size)
{
    using T = finenav_msgs::msg::Trajectory;
    if (traj.valid_fields & T::TRAJ_POSE)  traj.poses.resize(new_size);
    if (traj.valid_fields & T::TRAJ_VEL)   traj.twists.resize(new_size);
    if (traj.valid_fields & T::TRAJ_ACCEL) traj.accels.resize(new_size);
    if (traj.valid_fields & T::TRAJ_TIME)  traj.time_from_start.resize(new_size);
}

/**
 * @brief Convert a builtin_interfaces::msg::Duration to seconds (double).
 */
inline double duration_to_sec(const builtin_interfaces::msg::Duration& d)
{
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

/**
 * @brief Erase the first n elements from all populated fields of a trajectory.
 *        Caller is responsible for ensuring n <= current size.
 */
inline void erase_front(finenav_msgs::msg::Trajectory& traj, std::size_t n)
{
    using T = finenav_msgs::msg::Trajectory;
    if (n == 0) return;
    auto erase_n = [n](auto& vec) {
        vec.erase(vec.begin(), vec.begin() + static_cast<std::ptrdiff_t>(n));
    };
    if (traj.valid_fields & T::TRAJ_POSE)  erase_n(traj.poses);
    if (traj.valid_fields & T::TRAJ_VEL)   erase_n(traj.twists);
    if (traj.valid_fields & T::TRAJ_ACCEL) erase_n(traj.accels);
    if (traj.valid_fields & T::TRAJ_TIME)  erase_n(traj.time_from_start);
}

} // namespace finenav::motion_decorator::detail

```

---

### 📄 finenav_motion_decorator/include/i_motion_decorator.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_msgs/msg/trajectory.hpp"

namespace finenav {

/**
 * @brief Interface for stateless motion decorators.
 * Modifies the trajectory in-place to avoid copies.
 */
class IMotionDecorator {
public:
    virtual ~IMotionDecorator() = default;

    /**
     * @brief Apply the decorator to the trajectory.
     * @param trajectory The trajectory to be modified in-place.
     * @note Only throw when there is lethal failure
     */
    virtual void operator()(finenav_msgs::msg::Trajectory& trajectory) const = 0;
};

/**
 * @brief Enable pipe syntax for decorators: traj | DecoratorA() | DecoratorB()
 */
inline finenav_msgs::msg::Trajectory& operator|(
    finenav_msgs::msg::Trajectory& trajectory,
    const IMotionDecorator& decorator)
{
    decorator(trajectory);
    return trajectory;
}

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/include/seek_to_nearest_point.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"
#include <Eigen/Core>

namespace finenav {

/**
 * @brief Stateless decorator that advances the trajectory start to the waypoint
 *        nearest to a given reference position, discarding all preceding points.
 *
 * "Seek" mirrors file-seek semantics: the read head is repositioned to the
 * nearest entry rather than truncating the tail.
 *
 * Nearest-point search is performed in the XY plane by default (use_2d = true),
 * which is more robust on uneven terrain. Pass use_2d = false to use full 3-D
 * Euclidean distance instead.
 *
 * Requires TRAJ_POSE to be populated.
 *
 * - Silently no-ops if the trajectory is empty or the nearest point is already
 *   the first waypoint.
 * - Throws std::runtime_error if TRAJ_POSE is not set (programmer error).
 */
class SeekToNearestPoint : public IMotionDecorator {
public:
    /**
     * @brief Construct a SeekToNearestPoint decorator.
     * @param reference  Reference position in the map frame.
     * @param use_2d     If true (default), only XY distance is used for the
     *                   nearest-point search; Z is ignored.
     */
    explicit SeekToNearestPoint(Eigen::Vector3d reference, bool use_2d = true);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    Eigen::Vector3d reference_;
    bool use_2d_;
};

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/include/trim_by_aabb.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"
#include <Eigen/Geometry>

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory by discarding all waypoints
 *        that lie outside an axis-aligned bounding box (AABB).
 *
 * The decorator scans the trajectory from the front and retains only the
 * contiguous prefix of waypoints that are contained within the box.
 * The first waypoint found outside the box — and all following waypoints —
 * are removed.
 *
 * Although the bounding box is 3-D (Eigen::AlignedBox3d), a 2-D XY-only
 * check can be achieved by passing infinite Z extents:
 * @code
 *   Eigen::AlignedBox3d box;
 *   box.min() = {map_min_x, map_min_y, -std::numeric_limits<double>::infinity()};
 *   box.max() = {map_max_x, map_max_y,  std::numeric_limits<double>::infinity()};
 * @endcode
 *
 * Requires TRAJ_POSE to be populated.
 *
 * - Silently no-ops if all waypoints are inside the box.
 * - Throws std::runtime_error if TRAJ_POSE is not set (programmer error).
 */
class TrimByAABB : public IMotionDecorator {
public:
    /**
     * @brief Construct a TrimByAABB decorator.
     * @param bounds  3-D axis-aligned bounding box. Waypoints outside this box
     *                (first occurrence onwards) are removed.
     * @throws std::invalid_argument if bounds is empty (min > max on any axis).
     */
    explicit TrimByAABB(Eigen::AlignedBox3d bounds);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    Eigen::AlignedBox3d bounds_;
};

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/include/trim_by_count.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"
#include <cstddef>

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory to at most `max_points`
 *        waypoints, counting from the beginning of the path.
 *
 * All populated fields (poses, twists, accels, time_from_start) are resized
 * consistently according to the trajectory's valid_fields mask.
 *
 * - Silently no-ops when the trajectory already has <= max_points waypoints.
 * - Throws std::invalid_argument if max_points == 0 at construction time.
 */
class TrimByCount : public IMotionDecorator {
public:
    /** @brief Construct a TrimByCount decorator.
     *  @param max_points  Maximum number of waypoints to keep (must be > 0).
     *  @throws std::invalid_argument if max_points == 0.
     */
    explicit TrimByCount(std::size_t max_points);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    std::size_t max_points_;
};

} // namespace finenav
```

---

### 📄 finenav_motion_decorator/include/trim_by_distance.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory so that the cumulative
 *        arc-length along the path does not exceed `max_distance_m`.
 *
 * Distance is computed as the sum of consecutive Euclidean distances between
 * pose positions (x, y, z). Requires the trajectory to have TRAJ_POSE populated.
 *
 * - Silently no-ops if the total path length is already within the limit.
 * - Throws std::invalid_argument if max_distance_m <= 0 at construction time.
 * - Throws std::runtime_error if TRAJ_POSE is not set on the trajectory (programmer error).
 */
class TrimByDistance : public IMotionDecorator {
public:
    /** @brief Construct a TrimByDistance decorator.
     *  @param max_distance_m  Maximum cumulative arc-length to keep (metres, must be > 0).
     *  @throws std::invalid_argument if max_distance_m <= 0.
     */
    explicit TrimByDistance(double max_distance_m);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    double max_distance_m_;
};

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/include/trim_by_duration.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "i_motion_decorator.hpp"

namespace finenav {

/**
 * @brief Stateless decorator that trims a trajectory to only waypoints whose
 *        time_from_start is strictly less than or equal to `max_duration_sec`.
 *
 * Requires the trajectory to have TRAJ_TIME populated.
 *
 * - Silently no-ops if the trajectory is already within the limit.
 * - Throws std::invalid_argument if max_duration_sec <= 0 at construction time.
 * - Throws std::runtime_error if TRAJ_TIME is not set on the trajectory (programmer error).
 */
class TrimByDuration : public IMotionDecorator {
public:
    /** @brief Construct a TrimByDuration decorator.
     *  @param max_duration_sec  Maximum time-from-start to keep (seconds, must be > 0).
     *  @throws std::invalid_argument if max_duration_sec <= 0.
     */
    explicit TrimByDuration(double max_duration_sec);

    void operator()(finenav_msgs::msg::Trajectory& trajectory) const override;

private:
    double max_duration_sec_;
};

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_motion_decorator</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake_auto</buildtool_depend>

    <depend>finenav_msgs</depend>
    <depend>eigen3_cmake_module</depend>
    <depend>Eigen3</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_motion_decorator/src/seek_to_nearest_point.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "seek_to_nearest_point.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>
#include <limits>
#include <cstddef>

namespace finenav {

SeekToNearestPoint::SeekToNearestPoint(Eigen::Vector3d reference, bool use_2d)
    : reference_(std::move(reference)), use_2d_(use_2d)
{}

void SeekToNearestPoint::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_POSE)) {
        throw std::runtime_error(
            "SeekToNearestPoint: trajectory does not have TRAJ_POSE populated");
    }

    if (traj.poses.empty()) {
        return;
    }

    std::size_t nearest_idx = 0;
    double min_dist_sq = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& p = traj.poses[i].position;
        const double dx = p.x - reference_.x();
        const double dy = p.y - reference_.y();
        const double dz = use_2d_ ? 0.0 : (p.z - reference_.z());
        const double dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }

    // Silently no-op: nearest point is already the first waypoint
    if (nearest_idx == 0) {
        return;
    }

    motion_decorator::detail::erase_front(traj, nearest_idx);
}

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/src/trim_by_aabb.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_aabb.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>

namespace finenav {

TrimByAABB::TrimByAABB(Eigen::AlignedBox3d bounds)
    : bounds_(std::move(bounds))
{
    if (bounds_.isEmpty()) {
        throw std::invalid_argument("TrimByAABB: bounds must not be empty (min > max on some axis)");
    }
}

void TrimByAABB::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_POSE)) {
        throw std::runtime_error(
            "TrimByAABB: trajectory does not have TRAJ_POSE populated");
    }

    // Find the first waypoint outside the bounding box; keep [0, keep).
    std::size_t keep = traj.poses.size(); // default: keep all
    for (std::size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& p = traj.poses[i].position;
        const Eigen::Vector3d pt(p.x, p.y, p.z);
        if (!bounds_.contains(pt)) {
            keep = i;
            break;
        }
    }

    // Silently no-op: all waypoints are inside the box
    if (keep >= traj.poses.size()) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, keep);
}

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/src/trim_by_count.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_count.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>

namespace finenav {

TrimByCount::TrimByCount(std::size_t max_points) : max_points_(max_points)
{
    if (max_points_ == 0) {
        throw std::invalid_argument("TrimByCount: max_points must be > 0");
    }
}

void TrimByCount::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    // Determine current size from the first populated field
    std::size_t current_size = 0;
    if      (traj.valid_fields & T::TRAJ_POSE) current_size = traj.poses.size();
    else if (traj.valid_fields & T::TRAJ_VEL)  current_size = traj.twists.size();
    else if (traj.valid_fields & T::TRAJ_TIME) current_size = traj.time_from_start.size();

    // Silently no-op: already within the limit or trajectory is empty
    if (current_size <= max_points_) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, max_points_);
}

} // namespace finenav
```

---

### 📄 finenav_motion_decorator/src/trim_by_distance.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_distance.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>
#include <cmath>

namespace finenav {

TrimByDistance::TrimByDistance(double max_distance_m) : max_distance_m_(max_distance_m)
{
    if (max_distance_m_ <= 0.0) {
        throw std::invalid_argument("TrimByDistance: max_distance_m must be > 0");
    }
}

void TrimByDistance::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_POSE)) {
        throw std::runtime_error(
            "TrimByDistance: trajectory does not have TRAJ_POSE populated");
    }

    if (traj.poses.size() < 2) {
        return;  // 0 or 1 point — no arc-length to compute, nothing to trim
    }

    // Accumulate arc-length; stop at the first point that would exceed the limit
    double accumulated = 0.0;
    std::size_t keep = traj.poses.size();  // default: keep all

    for (std::size_t i = 1; i < traj.poses.size(); ++i) {
        const auto& prev = traj.poses[i - 1].position;
        const auto& curr = traj.poses[i].position;

        const double dx = curr.x - prev.x;
        const double dy = curr.y - prev.y;
        const double dz = curr.z - prev.z;
        accumulated += std::sqrt(dx * dx + dy * dy + dz * dz);

        if (accumulated > max_distance_m_) {
            keep = i;  // keep points [0, i), i.e. up to but not including i
            break;
        }
    }

    // Silently no-op: total path is within the limit
    if (keep >= traj.poses.size()) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, keep);
}

} // namespace finenav

```

---

### 📄 finenav_motion_decorator/src/trim_by_duration.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "trim_by_duration.hpp"
#include "detail/trim_utils.hpp"
#include <stdexcept>
#include <algorithm>

namespace finenav {

TrimByDuration::TrimByDuration(double max_duration_sec) : max_duration_sec_(max_duration_sec)
{
    if (max_duration_sec_ <= 0.0) {
        throw std::invalid_argument("TrimByDuration: max_duration_sec must be > 0");
    }
}

void TrimByDuration::operator()(finenav_msgs::msg::Trajectory& traj) const
{
    using T = finenav_msgs::msg::Trajectory;

    if (!(traj.valid_fields & T::TRAJ_TIME)) {
        throw std::runtime_error(
            "TrimByDuration: trajectory does not have TRAJ_TIME populated");
    }

    if (traj.time_from_start.empty()) {
        return;  // nothing to trim
    }

    // Find the first index whose time_from_start exceeds the limit
    std::size_t keep = traj.time_from_start.size();
    for (std::size_t i = 0; i < traj.time_from_start.size(); ++i) {
        if (motion_decorator::detail::duration_to_sec(traj.time_from_start[i]) > max_duration_sec_) {
            keep = i;
            break;
        }
    }

    // Silently no-op: all points are already within the limit
    if (keep >= traj.time_from_start.size()) {
        return;
    }

    motion_decorator::detail::resize_trajectory(traj, keep);
}

} // namespace finenav

```

---

### 📄 finenav_msgs/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/Trajectory.msg
        msg/RobotState.msg
        msg/NavStatus.msg
        srv/SetMap.srv
        action/ComputePlan.action
        action/NavigateToPose.action
        DEPENDENCIES geometry_msgs std_msgs nav_msgs
)

ament_auto_package()

```

---

### 📄 finenav_msgs/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_msgs</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake_auto</buildtool_depend>
    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>

    <depend>geometry_msgs</depend>
    <depend>std_msgs</depend>
    <depend>action_msgs</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_navigator/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_navigator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(finenav_localizer REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
        rclcpp_action
        ament_index_cpp
        behaviortree_cpp
        generate_parameter_library
        eigen3_cmake_module
        geometry_msgs
        finenav_msgs
        finenav_util
        finenav_localizer
)

# Library
generate_parameter_library(
        navigator_params
        src/finenav_navigator.yaml
)

add_library(finenav_navigator STATIC
        src/naviagtor.cpp
        src/behavior_tree_engine.cpp
)
ament_target_dependencies(finenav_navigator
        ${dependencies}
)
target_link_libraries(finenav_navigator
        navigator_params
        Eigen3::Eigen
)

# Action Plugins
add_library(finenav_compute_plan_behavior SHARED
        src/plugins/action/compute_plan.cpp
)
ament_target_dependencies(finenav_compute_plan_behavior
        ${dependencies}
)
target_compile_definitions(finenav_compute_plan_behavior PRIVATE BT_PLUGIN_EXPORT)

# Condition Plugins (pure BT nodes, no ROS params needed)
add_library(finenav_navigator_conditions SHARED
        src/plugins/condition/conditions.cpp
)
ament_target_dependencies(finenav_navigator_conditions
        ${dependencies}
)
target_compile_definitions(finenav_navigator_conditions PRIVATE BT_PLUGIN_EXPORT)

# PublishZeroCmdVel Plugin
add_library(finenav_publish_zero_cmd_vel_behavior SHARED
        src/plugins/action/publish_zero_cmd_vel.cpp
)
ament_target_dependencies(finenav_publish_zero_cmd_vel_behavior
        ${dependencies}
)
target_compile_definitions(finenav_publish_zero_cmd_vel_behavior PRIVATE BT_PLUGIN_EXPORT)

# Install
install(TARGETS finenav_navigator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS finenav_compute_plan_behavior finenav_navigator_conditions finenav_publish_zero_cmd_vel_behavior
        ARCHIVE DESTINATION lib/plugins
        LIBRARY DESTINATION lib/plugins
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

# Export
ament_export_include_directories(include)
ament_export_libraries(finenav_navigator)
ament_export_dependencies(${dependencies} Eigen3)

ament_package()
```

---

### 📄 finenav_navigator/behavior_trees/navigate_to_pose.xml

```xml
<!--
  ~ Copyright (c) 2026.
  ~ IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
  ~ All rights reserved.
  -->

<!--
  Navigate to pose with goal-reached detection.
  Uses ReactiveFallback: if IsGoalReached succeeds → tree succeeds and ComputePlan is halted.

  action_name: full path to the ControlLayer action server.
  planner_id:  must match AlgoConfigurator<T>::name of the planner loaded in the ControlLayer.
  -->
<root BTCPP_format="4">

    <!-- 基础导航：持续追踪，到达目标后发布零速并成功退出 -->
    <BehaviorTree ID="NavigateToPose">
        <Sequence name="TrackThenStop">
            <ReactiveFallback name="NavigateOrSucceed">
                <IsGoalReached  robot_pose="{robot_pose}"
                                robot_twist="{robot_twist}"
                                goal="{goal}"
                                tolerance="0.25"
                                speed_tolerance="0.1"/>
                <ComputePlan    action_name="/hello_finenav/mppi_layer"
                                goal="{goal}"
                                planner_id="finenav_mppi_controller"/>
            </ReactiveFallback>
            <!-- ReactiveFallback 返回 SUCCESS（IsGoalReached触发）后，发布零速 -->
            <PublishZeroCmdVel  topic_name="/cmd_vel"/>
        </Sequence>
    </BehaviorTree>

    <!-- 带卡死检测的导航：卡死时发布零速后整棵树 FAILURE -->
    <BehaviorTree ID="NavigateWithStuckDetection">
        <Sequence name="TrackThenStop">
            <ReactiveFallback name="NavigateOrSucceed">
                <IsGoalReached  robot_pose="{robot_pose}"
                                robot_twist="{robot_twist}"
                                goal="{goal}"
                                tolerance="0.25"
                                speed_tolerance="0.1"/>
                <ReactiveSequence name="TrackUnlessStuck">
                    <Inverter>
                        <IsStuck robot_pose="{robot_pose}"
                                 dist_threshold="0.05"
                                 timeout_s="5.0"/>
                    </Inverter>
                    <ComputePlan action_name="/hello_finenav/mppi_layer"
                                 goal="{goal}"
                                 planner_id="finenav_mppi_controller"/>
                </ReactiveSequence>
            </ReactiveFallback>
            <!-- ReactiveFallback 返回 SUCCESS（IsGoalReached触发）后，发布零速 -->
            <PublishZeroCmdVel  topic_name="/cmd_vel"/>
        </Sequence>
    </BehaviorTree>

</root>

```

---

### 📄 finenav_navigator/behavior_trees/test_tree.xml

```xml
<!--
  This Behavior Tree is used for rest.
  -->

<root BTCPP_format="4" >
    <BehaviorTree ID="TestComputePlan">
        <Sequence name="TestSequence">
            <ComputePlan   name="compute_plan"
                          goal="{goal}"
                          planner_id="SimpleTestPlanner"/>
        </Sequence>
    </BehaviorTree>
</root>
```

---

### 📄 finenav_navigator/include/finenav_navigator/behavior_tree_engine.hpp

```cpp
// Copyright (c) 2026.bt_action_node.hpp
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "finenav_navigator/thirdparty/ros_node_params.hpp"
#include "finenav_navigator/thirdparty/plugins.hpp"

namespace finenav {

class BehaviorTreeEngine {
  public:
    explicit BehaviorTreeEngine(rclcpp::Node::SharedPtr node); //const std::vector<std::string> & plugin_libraries,
    virtual ~BehaviorTreeEngine() = default;

    void registerPlugins(const std::vector<std::string>& plugins);

    void registerBehaviorTrees(const std::vector<std::string>& tree_dirs);

    bool loadPlugin(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path, BT::RosNodeParams params);

    bool loadBehaviorTree(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path);

    BT::BehaviorTreeFactory& factory();

  private:
    // Expect "[package_name]/.../..."
    static std::pair<std::string, std::string> parseParamPath(const std::string& input);

    rclcpp::Logger logger_;
    BT::BehaviorTreeFactory factory_;
    BT::RosNodeParams ros_params_;

};


}
```

---

### 📄 finenav_navigator/include/finenav_navigator/navigator.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "finenav_navigator/behavior_tree_engine.hpp"

#include "finenav_util/simple_action_server.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_msgs/action/navigate_to_pose.hpp"

#include "finenav_navigator/navigator_params.hpp"

namespace finenav {

/**
 * @brief Navigator — 行为树驱动的任务级导航器。
 *
 * 遵循框架 NodeThread 设计模式：
 *  - 内部持有私有子节点 pnode_（名称固定为 "finenav_navigator"）
 *  - pnode_ 由 NodeThread 在独立线程中 spin，对外部调用方完全透明
 *  - 构造函数完成全部初始化，无需二阶段 initialize()
 *
 * 使用前须调用 set_robot_state_provider() 注入位姿提供函数；
 * FineNavEngine::createControlLayer() 会自动完成注入。
 *
 * @param parent_node  宿主节点（用于获取 use_sim_time 及命名空间继承）
 */
class Navigator {
public:
    using ActionT            = finenav_msgs::action::NavigateToPose;
    using ActionServer       = SimpleActionServer<ActionT>;
    using RobotPoseProvider  = std::function<geometry_msgs::msg::Pose()>;
    using RobotTwistProvider = std::function<geometry_msgs::msg::Twist()>;

    explicit Navigator(rclcpp::Node::SharedPtr parent_node);

    // 禁止复制
    Navigator(const Navigator&)            = delete;
    Navigator& operator=(const Navigator&) = delete;

    ~Navigator() = default;

    /**
     * @brief 注入机器人位姿提供函数。
     *
     * 每次 BT tick 前调用，将最新位姿写入 Blackboard ("robot_pose")，
     * 供 IsGoalReached、IsStuck 等条件节点读取。
     * 由 FineNavEngine 在构造时自动注入。
     */
    void set_robot_pose_provider(RobotPoseProvider provider) {
        robot_pose_provider_ = std::move(provider);
    }

    /**
     * @brief 注入机器人速度提供函数。
     *
     * 每次 BT tick 前调用，将最新速度写入 Blackboard ("robot_twist")，
     * 供 IsGoalReached 等条件节点读取。
     * 由 FineNavEngine 在构造时自动注入。
     */
    void set_robot_twist_provider(RobotTwistProvider provider) {
        robot_twist_provider_ = std::move(provider);
    }

private:
    void execute();

    // ── ROS 基础设施 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr       pnode_;
    std::unique_ptr<NodeThread>   pnode_thread_;

    // ── 业务组件 ──────────────────────────────────────────────────────────────
    std::unique_ptr<BehaviorTreeEngine> bt_engine_;
    std::unique_ptr<ActionServer>       action_server_;

    finenav_navigator::Params                         params_;
    std::unique_ptr<finenav_navigator::ParamListener> param_listener_;

    RobotPoseProvider  robot_pose_provider_;
    RobotTwistProvider robot_twist_provider_;

    BT::Tree tree_;
};

} // namespace finenav

```

---

### 📄 finenav_navigator/include/finenav_navigator/plugins/action/compute_plan.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_navigator/thirdparty/bt_action_node.hpp"

namespace finenav {

/**
 * @brief A behavior tree ActionNode that requests a plan from finenav_msgs::action::ComputePlan action server
 */
class ComputePlanAction : public BT::RosActionNode<finenav_msgs::action::ComputePlan> {
   public:
    ComputePlanAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::Pose>("goal", "Target pose for path planning"),
            BT::InputPort<std::string>("planner_id", "ID of the planner to use")
        });
    }

    bool setGoal(Goal& goal) override;

    BT::NodeStatus onResultReceived(const WrappedResult& result) override;

    /**
     * @brief Called each tick while RUNNING. Returns true when the blackboard
     * goal has changed since the last send, triggering an in-place resend
     * (no cancel — relies on ControlLayer's pending/preempt slot).
     */
    bool shouldResendGoal() override;

private:
    geometry_msgs::msg::Pose last_sent_goal_{};

    static bool posesEqual(const geometry_msgs::msg::Pose& a,
                           const geometry_msgs::msg::Pose& b,
                           double eps = 1e-4) // TODO: equal不应该是这样算的
    {
        return std::abs(a.position.x - b.position.x) < eps &&
               std::abs(a.position.y - b.position.y) < eps &&
               std::abs(a.position.z - b.position.z) < eps;
    }
};

}

```

---

### 📄 finenav_navigator/include/finenav_navigator/plugins/action/publish_zero_cmd_vel.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include "finenav_navigator/thirdparty/bt_topic_pub_node.hpp"

namespace finenav {

/**
 * @brief PublishZeroCmdVel — 向指定话题发布一条零速度指令（linear/angular 全为 0）。
 *
 * 作为 BT::RosTopicPubNode<Twist> 的同步动作节点，tick 一次即发布并立即返回 SUCCESS。
 * 典型用途：放置在导航成功/卡死检测后的 Sequence 末尾，确保机器人停止运动。
 *
 * 输入端口:
 *   - topic_name (string, 默认 "/cmd_vel"): 要发布的话题名称
 */
class PublishZeroCmdVel : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
    PublishZeroCmdVel(const std::string& name,
                      const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
        : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        // 继承 topic_name 端口；这里不添加额外端口
        return providedBasicPorts({});
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) override
    {
        // 零速：全部分量保持默认值 0.0
        msg = geometry_msgs::msg::Twist{};
        return true;
    }
};

} // namespace finenav

```

---

### 📄 finenav_navigator/include/finenav_navigator/plugins/condition/is_goal_reached.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsGoalReached — 检查机器人是否已到达目标（位置 + 速度双重判定）。
 *
 * Blackboard 输入:
 *   - robot_pose  (geometry_msgs::msg::Pose):  当前机器人位姿
 *   - robot_twist (geometry_msgs::msg::Twist): 当前机器人速度
 *   - goal        (geometry_msgs::msg::Pose):  导航目标位姿
 *
 * 输入端口:
 *   - tolerance        (double, 默认 0.25 m):   到达判定的距离阈值
 *   - speed_tolerance  (double, 默认 0.1 m/s):  到达判定的合速度阈值
 *                      设为 0.0 可禁用速度检查（仅判断位置）
 *
 * 仅当位置距离 < tolerance 且合速度 < speed_tolerance 时返回 SUCCESS。
 */
class IsGoalReached : public BT::ConditionNode {
public:
    IsGoalReached(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose> ("robot_pose",      "{robot_pose}",  "Current robot pose"),
            BT::InputPort<geometry_msgs::msg::Twist>("robot_twist",     "{robot_twist}", "Current robot twist"),
            BT::InputPort<geometry_msgs::msg::Pose> ("goal",            "{goal}",        "Goal pose"),
            BT::InputPort<double>("tolerance",       0.25, "Goal position tolerance [m]"),
            BT::InputPort<double>("speed_tolerance", 0.1,  "Goal speed tolerance [m/s]; set 0 to disable"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose robot_pose, goal;
        geometry_msgs::msg::Twist robot_twist;
        double tolerance       = 0.25;
        double speed_tolerance = 0.1;

        if (!getInput("robot_pose", robot_pose) || !getInput("goal", goal)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("tolerance",       tolerance);
        getInput("speed_tolerance", speed_tolerance);

        // 位置检查
        const double dx   = robot_pose.position.x - goal.position.x;
        const double dy   = robot_pose.position.y - goal.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist >= tolerance) {
            return BT::NodeStatus::FAILURE;
        }

        // 速度检查（speed_tolerance <= 0 时跳过）
        if (speed_tolerance > 0.0) {
            if (!getInput("robot_twist", robot_twist)) {
                // 速度未注入时降级为仅位置判断
                return BT::NodeStatus::SUCCESS;
            }
            const double vx    = robot_twist.linear.x;
            const double vy    = robot_twist.linear.y;
            const double speed = std::sqrt(vx * vx + vy * vy);
            if (speed >= speed_tolerance) {
                return BT::NodeStatus::FAILURE;
            }
        }

        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace finenav

```

---

### 📄 finenav_navigator/include/finenav_navigator/plugins/condition/is_stuck.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsStuck — 检测机器人是否在一段时间内没有充分移动（卡死）。
 *
 * Blackboard 输入:
 *   - robot_pose (geometry_msgs::msg::Pose): 当前机器人位姿
 *
 * 输入端口:
 *   - dist_threshold (double, 默认 0.05 m): 时间窗内最小位移阈值
 *   - timeout_s      (double, 默认 5.0 s):  触发卡死判定的时间窗长度
 *
 * 返回 SUCCESS 表示已卡死，FAILURE 表示正常移动中。
 * halt() 重置所有内部状态（适用于恢复行为后重新检测）。
 */
// TODO: 可能需要重构，需要参考BT4库提供的文档
// Claude的输出
// BT::ConditionNode::halt() 是 final 的，不能被覆写。改用 SimpleConditionNode 的机制，或通过 setPreTickFunction 替代。实际上 BT4
// 的 ConditionNode 不支持自定义 halt，要用 StatefulActionNode 或继承 BT::ConditionNode 并在构造时注册。
// 最干净的解法：将 IsStuck 改为继承 BT::SimpleConditionNode 的函数对象，或用 BT::ConditionNode 并把状态重置放在 tick 逻辑里
// （检测到自身被重新激活时）。由于 halt 不可覆写，IsStuck 改为在每次 tick 时检测自己是否是首次被 tick（通过时间戳或标志位的特殊值）：

class IsStuck : public BT::ConditionNode {
public:
    IsStuck(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config), ref_set_(false) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose>("robot_pose",      "{robot_pose}", "Current robot pose"),
            BT::InputPort<double>("dist_threshold", 0.05,  "Minimum displacement [m] within window"),
            BT::InputPort<double>("timeout_s",      5.0,   "Time window [s] for stuck detection"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose robot_pose;
        if (!getInput("robot_pose", robot_pose)) {
            return BT::NodeStatus::FAILURE;
        }
        double dist_threshold = 0.05;
        double timeout_s      = 5.0;
        getInput("dist_threshold", dist_threshold);
        getInput("timeout_s",      timeout_s);

        const double cur_x = robot_pose.position.x;
        const double cur_y = robot_pose.position.y;
        const auto now = std::chrono::steady_clock::now();

        // BT::ConditionNode::halt() is final — detect "cold start" by checking
        // whether the node was last ticked more than timeout_s ago (i.e. it was
        // dormant / freshly reactivated).  In that case, reset the window.
        if (ref_set_) {
            const double gap =
                std::chrono::duration<double>(now - last_tick_time_).count();
            if (gap > timeout_s) {
                ref_set_ = false; // dormant long enough — treat as fresh start
            }
        }
        last_tick_time_ = now;

        if (!ref_set_) {
            ref_x_        = cur_x;
            ref_y_        = cur_y;
            window_start_ = now;
            ref_set_      = true;
            return BT::NodeStatus::FAILURE;
        }

        const double dx    = cur_x - ref_x_;
        const double dy    = cur_y - ref_y_;
        const double moved = std::sqrt(dx * dx + dy * dy);

        if (moved > dist_threshold) {
            ref_x_        = cur_x;
            ref_y_        = cur_y;
            window_start_ = now;
            return BT::NodeStatus::FAILURE;
        }

        const double elapsed =
            std::chrono::duration<double>(now - window_start_).count();
        return (elapsed > timeout_s) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    bool   ref_set_{false};
    double ref_x_{0.0}, ref_y_{0.0};
    std::chrono::steady_clock::time_point window_start_;
    std::chrono::steady_clock::time_point last_tick_time_;
};

} // namespace finenav


```

---

### 📄 finenav_navigator/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_navigator</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>rclcpp</depend>
    <depend>rclcpp_action</depend>
    <depend>ament_index_cpp</depend>
    <depend>behaviortree_cpp</depend>
    <depend>generate_parameter_library</depend>
    <depend>eigen</depend>
    <depend>eigen3_cmake_module</depend>
    <depend>finenav_msgs</depend>
    <depend>finenav_util</depend>
    <depend>finenav_localizer</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_navigator/src/behavior_tree_engine.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/behavior_tree_engine.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace finenav {

BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Node::SharedPtr node)
  : logger_(node->get_logger())
{
    // 初始化 RosNodeParams，这是 BT.ROS2 依赖注入的核心
    ros_params_.nh = node;
    ros_params_.default_port_value = ""; // 默认不指定 topic，强制 XML 指定
    ros_params_.server_timeout = std::chrono::milliseconds(1000);
    ros_params_.wait_for_server_timeout = std::chrono::milliseconds(500);
}

void BehaviorTreeEngine::registerPlugins(const std::vector<std::string>& plugins)
{

    RCLCPP_INFO(logger_, "FineNav Engine: Loading plugins from %zu sources...", plugins.size());

    for (const auto& plugin_param : plugins) {
        const auto [pkg_name, sub_folder] = parseParamPath(plugin_param);
        if (pkg_name.empty()) { continue; } // skip invalid plugins directories

        std::filesystem::path plugin_directory;
        try {
            // Plugins are usually installed in "[ws]/install/[pkg_name]/lib/"
            const auto prefix = ament_index_cpp::get_package_prefix(pkg_name);
            plugin_directory = std::filesystem::path(prefix) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            RCLCPP_ERROR(logger_, "Plugin loading failed: Package '%s' not found.", pkg_name.c_str());
            continue;
        }
        if (!std::filesystem::exists(plugin_directory)) {
            RCLCPP_WARN(logger_, "Plugin directory not found: %s", plugin_directory.c_str());
            continue;
        }

        // Discovery and load .so
        using std::filesystem::recursive_directory_iterator;
        int loaded_count = 0;
        for(const auto& entry : recursive_directory_iterator(plugin_directory))
        {
            if(entry.path().extension() == ".so")
            {
                if (loadPlugin(factory_, entry.path(), ros_params_)) {
                    loaded_count++;
                }
            }
        }
        RCLCPP_INFO(logger_, "  -> Scanned '%s': Loaded %d plugins.", plugin_directory.c_str(), loaded_count);
    }
}

void BehaviorTreeEngine::registerBehaviorTrees(const std::vector<std::string>& tree_dirs)
{
    for (const auto& tree_dir_param : tree_dirs) {
        const auto [pkg_name, sub_folder] = parseParamPath(tree_dir_param);
        if (pkg_name.empty()) { continue; } // skip invalid plugins directories

        std::filesystem::path tree_directory;
        try {
            // BehaviorTress are usually installed in "[ws]/install/[pkg_name]/share/[pkg_name]/"
            const auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
            tree_directory = std::filesystem::path(share_dir) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            RCLCPP_ERROR(logger_, "BT loading failed: Package '%s' not found.", pkg_name.c_str());
            continue;
        }
        if (!std::filesystem::exists(tree_directory)) {
            RCLCPP_WARN(logger_, "Plugin directory not found: %s", tree_directory.c_str());
            continue;
        }

        // Discovery and load .xml
        using std::filesystem::recursive_directory_iterator;
        int loaded_count = 0;
        for(const auto& entry : recursive_directory_iterator(tree_directory))
        {
            if (loadBehaviorTree(factory_, entry.path())) {
                loaded_count++;
            }
        }
        RCLCPP_INFO(logger_, "  -> Scanned '%s': Loaded %d trees.", tree_directory.c_str(), loaded_count);
    }
}

bool BehaviorTreeEngine::loadPlugin(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path,
    BT::RosNodeParams params) {

    const auto filename = file_path.filename();
    try
    {
        BT::SharedLibrary loader(file_path.string());
        if(loader.hasSymbol(BT::PLUGIN_SYMBOL))
        {
            typedef void (*Func)(BT::BehaviorTreeFactory&);
            auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
            func(factory);
        }
        else if(loader.hasSymbol(BT::ROS_PLUGIN_SYMBOL))
        {
            typedef void (*Func)(BT::BehaviorTreeFactory&, const BT::RosNodeParams&);
            auto func = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
            func(factory, params);
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to load Plugin from file: %s.", filename.c_str());
            return false;
        }
        RCLCPP_INFO(logger_, "Loaded ROS Plugin: %s", filename.c_str());
        return true;
    }
    catch(const std::exception& ex)
    {
        RCLCPP_ERROR(logger_, "Failed to load ROS Plugin: %s \n %s", filename.c_str(),
                     ex.what());
        return false;
    }
}

bool BehaviorTreeEngine::loadBehaviorTree(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path) {

    try
    {
        factory.registerBehaviorTreeFromFile(file_path.string());
        RCLCPP_INFO(logger_, "Loaded BehaviorTree: %s", file_path.filename().c_str());
        return true;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "Failed to load BehaviorTree: %s \n %s",
                     file_path.filename().c_str(), e.what());
        return false;
    }
}

BT::BehaviorTreeFactory& BehaviorTreeEngine::factory() {
    return factory_;
}

std::pair<std::string, std::string> BehaviorTreeEngine::parseParamPath(const std::string& input) {
    auto pos = input.find('/');
    if (pos == std::string::npos) {
        return {input, ""}; // only package name
    }
    return {input.substr(0, pos), input.substr(pos + 1)};
}

}
```

---

### 📄 finenav_navigator/src/finenav_navigator.yaml

```yaml
finenav_navigator:
  action_name: {
    type: string,
    default_value: "navigate_to_pose",
    description: "The name the Action Server takes requests from",
  }
  tick_frequency: {
    type: int,
    default_value: 100,
    description: "Frequency in Hz to tick() the Behavior tree at",
    validation: {
      bounds<>: [1, 1000]
    }
  }
  plugins: {
    type: string_array,
    default_value: [],
    description: "List of 'package_name/subfolder' containing BehaviorTree plugins to load into the factory",
    validation: {
      unique<>: null,
    }
  }
  ros_plugins_timeout: {
    type: int,
    default_value: 1000,
    description: "Timeout (ms) used in BT::RosNodeParams",
    validation: {
      bounds<>: [1, 10000]
    }
  }
  behavior_trees: {
    type: string_array,
    default_value: [],
    description: "List of 'package_name/subfolder' containing SubTrees to load into the BehaviorTree factory",
    validation: {
      unique<>: null,
    }
  }
```

---

### 📄 finenav_navigator/src/naviagtor.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/navigator.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace finenav {

Navigator::Navigator(rclcpp::Node::SharedPtr parent_node)
{
    // ── 创建私有子节点（参数前缀由 yaml 的顶层 key "finenav_navigator" 决定）──
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        parent_node->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_navigator",            // 节点名称，与 yaml 顶层 key 一致
        parent_node->get_name(),        // 命名空间继承宿主节点名
        pnode_opts);

    // ── 读取参数 ──────────────────────────────────────────────────────────────
    param_listener_ = std::make_unique<finenav_navigator::ParamListener>(pnode_);
    params_ = param_listener_->get_params();

    // ── 创建 Action Server ────────────────────────────────────────────────────
    action_server_ = std::make_unique<ActionServer>(
        pnode_,
        params_.action_name,
        std::bind(&Navigator::execute, this));

    // ── 初始化行为树引擎 ──────────────────────────────────────────────────────
    bt_engine_ = std::make_unique<BehaviorTreeEngine>(pnode_);
    if (!params_.plugins.empty())       { bt_engine_->registerPlugins(params_.plugins); }
    if (!params_.behavior_trees.empty()){ bt_engine_->registerBehaviorTrees(params_.behavior_trees); }

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Initialized and spinning.");
}

void Navigator::execute()
{
    RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Executing action...");

    auto goal = action_server_->get_current_goal();

    try {
        tree_ = bt_engine_->factory().createTree(goal->behavior_tree);
    } catch (std::exception& ex) {
        RCLCPP_ERROR(pnode_->get_logger(),
            "[Navigator] Failed to create tree [%s]: %s",
            goal->behavior_tree.c_str(), ex.what());
        action_server_->terminate_all();
        return;
    }

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("goal", goal->pose.pose);

    rclcpp::WallRate loop_rate(params_.tick_frequency);
    RCLCPP_INFO(pnode_->get_logger(),
        "[Navigator] Starting navigation to (%.2f, %.2f)",
        goal->pose.pose.position.x, goal->pose.pose.position.y);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Cancellation requested.");
            tree_.haltTree();
            action_server_->terminate_all();
            return;
        }

        if (action_server_->is_preempt_requested()) {
            RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Preemption requested.");
            tree_.haltTree();
            return;
        }

        // 每 tick 前将最新机器人位姿和速度注入 Blackboard，供条件节点读取
        if (robot_pose_provider_) {
            blackboard->set<geometry_msgs::msg::Pose>("robot_pose", robot_pose_provider_());
        }
        if (robot_twist_provider_) {
            blackboard->set<geometry_msgs::msg::Twist>("robot_twist", robot_twist_provider_());
        }

        auto status = tree_.tickExactlyOnce();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(pnode_->get_logger(), "[Navigator] Navigation succeeded.");
            action_server_->succeeded_current();
            return;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(pnode_->get_logger(), "[Navigator] Navigation failed.");
            action_server_->terminate_all();
            return;
        }

        // TODO: 发布 Feedback

        loop_rate.sleep();
    }
}

} // namespace finenav
```

---

### 📄 finenav_navigator/src/plugins/action/compute_plan.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/compute_plan.hpp"

namespace finenav {

ComputePlanAction::ComputePlanAction(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosActionNode<finenav_msgs::action::ComputePlan>(name, conf, params) {}

bool ComputePlanAction::setGoal(Goal& goal) {
    if (!getInput("goal", goal.goal) ||
        !getInput("planner_id", goal.planner_id)) {
        RCLCPP_ERROR(logger(), "ComputePlan: missing input ports");
        return false;
    }
    // Record the goal being sent so shouldResendGoal() can detect changes.
    last_sent_goal_ = goal.goal;
    return true;
}

bool ComputePlanAction::shouldResendGoal() {
    geometry_msgs::msg::Pose current_goal;
    if (!getInput("goal", current_goal)) {
        return false;
    }
    if (!posesEqual(current_goal, last_sent_goal_)) {
        RCLCPP_INFO(logger(),
            "ComputePlan: goal changed (%.2f,%.2f)->(%.2f,%.2f), resending.",
            last_sent_goal_.position.x, last_sent_goal_.position.y,
            current_goal.position.x,   current_goal.position.y); // TODO： 只输出2维变化，感觉也不太合理
        return true;
    }
    return false;
}

BT::NodeStatus ComputePlanAction::onResultReceived(const WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        std::cout << "ComputePlanAction of "
                  << rclcpp_action::to_string(result.goal_id) << " succeeded." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

}

#include "finenav_navigator/thirdparty/plugins.hpp"
CreateRosNodePlugin(finenav::ComputePlanAction, "ComputePlan");
```

---

### 📄 finenav_navigator/src/plugins/action/publish_zero_cmd_vel.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/publish_zero_cmd_vel.hpp"
#include "finenav_navigator/thirdparty/plugins.hpp"

CreateRosNodePlugin(finenav::PublishZeroCmdVel, "PublishZeroCmdVel");

```

---

### 📄 finenav_navigator/src/plugins/condition/conditions.cpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/condition/is_goal_reached.hpp"
#include "finenav_navigator/plugins/condition/is_stuck.hpp"
#include "behaviortree_cpp/bt_factory.h"

// Pure BT plugin (no ROS node params needed) — uses BT::PLUGIN_SYMBOL
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<finenav::IsGoalReached>("IsGoalReached");
    factory.registerNodeType<finenav::IsStuck>("IsStuck");
}

```

---

### 📄 finenav_planner/finenav_mppi_controller/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.5)
project(finenav_mppi_controller)

add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 0)
set(XTENSOR_USE_XSIMD 1)

# set(XTENSOR_DEFAULT_LAYOUT column_major)  # row_major, column_major
# set(XTENSOR_DEFAULT_TRAVERSAL row_major)  # row_major, column_major

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)
include_directories(
  include
)


set(dependencies_pkgs
  rclcpp
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  angles
  OpenCV
        finenav_collision_model
        finenav_interface
        finenav_util
        finenav_core
        generate_parameter_library
        finenav_msgs
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()


# ↓ 覆盖 nav2_package() 设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=deprecated-declarations)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ↑ 覆盖 nav2_package() 设置

# ↓ 生成参数
generate_parameter_library(
        finenav_mppi_controller_params
        src/finenav_mppi_controller.yaml
)
# ↑ 生成参数

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

add_library(mppi_controller SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
)

add_library(mppi_critics SHARED
  src/critics/obstacles_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_align_legacy_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC ${xsimd_INCLUDE_DIRS})  #  ${OpenMP_INCLUDE_DIRS}
  target_link_libraries(${lib} xtensor xtensor::optimize xtensor::use_xsimd finenav_mppi_controller_params )
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

install(
  TARGETS ${libraries}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)




find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)


set(test_dependencies
        ${dependencies_pkgs}
        pcl_conversions
        pcl_msgs
)






install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
#  add_subdirectory(test)
  # add_subdirectory(benchmark)
endif()


ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)

ament_package()
```

---

### 📄 finenav_planner/finenav_mppi_controller/LICENSE.md

```markdown
MIT License

Copyright (c) 2021-2022 Fast Sense Studio
Copyright (c) 2022-2023 Samsung Research America
Copyright (c) 2023 Open Navigation LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

### 📄 finenav_planner/finenav_mppi_controller/benchmark/CMakeLists.txt

```
find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  optimizer_benchmark
  controller_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name}
    ${name}.cpp
  )
  ament_target_dependencies(${name}
    ${dependencies_pkgs}
  )
  target_link_libraries(${name}
    mppi_controller mppi_critics benchmark
  )

target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test/utils
)
endforeach()
```

---

### 📄 finenav_planner/finenav_mppi_controller/benchmark/controller_benchmark.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <benchmark/benchmark.h>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/controller.hpp"

#include "utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};

RosLockGuard g_rclcpp;

void prepareAndRunBenchmark(
  bool consider_footprint, std::string motion_model,
  std::vector<std::string> critics, benchmark::State & state)
{
  bool visualize = false;

  int batch_size = 300;
  int time_steps = 12;
  unsigned int path_points = 50u;
  int iteration_count = 2;
  double lookahead_distance = 10.0;

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  TestPose start_pose = costmap_settings.getCenterPose();
  double path_step = costmap_settings.resolution;

  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
    lookahead_distance, motion_model, consider_footprint};

  unsigned int offset = 4;
  unsigned int obstacle_size = offset * 2;

  unsigned char obstacle_cost = 250;

  auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

  obst_x = obst_x - offset;
  obst_y = obst_y - offset;
  addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});

  printInfo(optimizer_settings, path_settings, critics);

  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpControllerParams(visualize, params);
  setUpOptimizerParams(optimizer_settings, critics, params);
  options.parameter_overrides(params);
  auto node = getDummyNode(options);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  auto broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  auto map_odom_broadcaster = std::async(
    std::launch::async, sendTf, "map", "odom", broadcaster, node,
    20);

  auto odom_base_link_broadcaster = std::async(
    std::launch::async, sendTf, "odom", "base_link", broadcaster, node,
    20);

  auto controller = getDummyController(node, tf_buffer, costmap_ros);

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  controller->setPlan(path);

  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  for (auto _ : state) {
    controller->computeVelocityCommands(pose, velocity, dummy_goal_checker);
  }
  map_odom_broadcaster.wait();
  odom_base_link_broadcaster.wait();
}

static void BM_DiffDrivePointFootprint(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_DiffDrive(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}


static void BM_Omni(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Omni";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"TwirlingCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_Ackermann(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_GoalCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"GoalCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_GoalAngleCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"GoalAngleCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_ObstaclesCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"ObstaclesCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_ObstaclesCriticPointFootprint(benchmark::State & state)
{
  bool consider_footprint = false;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"ObstaclesCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_TwilringCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"TwirlingCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_PathFollowCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"PathFollowCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_PathAngleCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"PathAngleCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

BENCHMARK(BM_DiffDrivePointFootprint)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_DiffDrive)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Omni)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Ackermann)->Unit(benchmark::kMillisecond);

BENCHMARK(BM_GoalCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_GoalAngleCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PathAngleCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PathFollowCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_ObstaclesCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_ObstaclesCriticPointFootprint)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_TwilringCritic)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
```

---

### 📄 finenav_planner/finenav_mppi_controller/benchmark/optimizer_benchmark.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <benchmark/benchmark.h>
#include <string>

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>

#include <xtensor/xarray.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

#include "utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};

RosLockGuard g_rclcpp;

void prepareAndRunBenchmark(
  bool consider_footprint, std::string motion_model,
  std::vector<std::string> critics, benchmark::State & state)
{
  int batch_size = 300;
  int time_steps = 12;
  unsigned int path_points = 50u;
  int iteration_count = 2;
  double lookahead_distance = 10.0;

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  TestPose start_pose = costmap_settings.getCenterPose();
  double path_step = costmap_settings.resolution;

  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
    lookahead_distance, motion_model, consider_footprint};

  unsigned int offset = 4;
  unsigned int obstacle_size = offset * 2;

  unsigned char obstacle_cost = 250;

  auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

  obst_x = obst_x - offset;
  obst_y = obst_y - offset;
  addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});

  printInfo(optimizer_settings, path_settings, critics);
  auto node = getDummyNode(optimizer_settings, critics);
  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, parameters_handler.get());

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  for (auto _ : state) {
    optimizer->evalControl(pose, velocity, path, dummy_goal_checker);
  }
}

static void BM_DiffDrivePointFootprint(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_DiffDrive(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "DiffDrive";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}


static void BM_Omni(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Omni";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"TwirlingCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_Ackermann(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"GoalCritic"}, {"GoalAngleCritic"}, {"ObstaclesCritic"},
    {"PathAngleCritic"}, {"PathFollowCritic"}, {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_GoalCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"GoalCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_GoalAngleCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"GoalAngleCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_ObstaclesCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"ObstaclesCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_ObstaclesCriticPointFootprint(benchmark::State & state)
{
  bool consider_footprint = false;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"ObstaclesCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_TwilringCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"TwirlingCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_PathFollowCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"PathFollowCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_PathAngleCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "Ackermann";
  std::vector<std::string> critics = {{"PathAngleCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

BENCHMARK(BM_DiffDrivePointFootprint)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_DiffDrive)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Omni)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Ackermann)->Unit(benchmark::kMillisecond);

BENCHMARK(BM_GoalCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_GoalAngleCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PathAngleCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PathFollowCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_ObstaclesCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_ObstaclesCriticPointFootprint)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_TwilringCritic)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
```

---

### 📄 finenav_planner/finenav_mppi_controller/critics.xml

```xml
<class_libraries>
  <library path="mppi_critics">

    <class type="mppi::critics::ObstaclesCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for obstacle avoidance</description>
    </class>

    <class type="mppi::critics::CostCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for obstacle avoidance using costmap score</description>
    </class>

    <class type="mppi::critics::GoalCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for driving towards the goal</description>
    </class>

    <class type="mppi::critics::GoalAngleCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for achieving the goal heading angle</description>
    </class>

    <class type="mppi::critics::PathAlignCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for aligning to path</description>
    </class>

    <class type="mppi::critics::PathAlignLegacyCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for aligning to path (legacy)</description>
    </class>

    <class type="mppi::critics::PathAngleCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for tracking the path in the correct heading</description>
    </class>

    <class type="mppi::critics::PathFollowCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for driving towards the goal that is furthest among trajectories nearest path points</description>
    </class>

    <class type="mppi::critics::PreferForwardCritic" base_class_type="mppi::critics::CriticFunction">
      <description></description>
    </class>

    <class type="mppi::critics::TwirlingCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for preventing twirling behavior when using omnidirectional models</description>
    </class>

    <class type="mppi::critics::ConstraintCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for incentivizing moving within kinematic and dynamic bounds</description>
    </class>

    <class type="mppi::critics::VelocityDeadbandCritic" base_class_type="mppi::critics::CriticFunction">
      <description>mppi critic for restricting command velocities in deadband range</description>
    </class>

  </library>
</class_libraries>
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/controller.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <string>
#include <memory>

//#include "finenav_mppi_controller/tools/path_handler.hpp"
//#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

#include "rclcpp/rclcpp.hpp"


#include "finenav_interface/test_map_params.hpp"
#include "finenav_interface/test_planner_params.hpp"

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */


class MPPIController
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
    MPPIController() = default;

    static constexpr finenav::IOProfile InputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr finenav::IOProfile OutputProfile = {
        .pose = true,
        .vel  = true,
        .accel= false,
        .time = false
    };

    using IMapView = nav2_mppi_controller::IMapView;
    using Context = finenav::PlanningContext<MPPIController>;
    using ConfigType = finenav_mppi_controller::Params;
    void configure(const ConfigType& params) {              //应该传入参数 因测试先去除

        registerCritic(std::make_unique<critics::ConstraintCritic>());
        registerCritic(std::make_unique<critics::CostCritic>());
        registerCritic(std::make_unique<critics::GoalAngleCritic>());
        registerCritic(std::make_unique<critics::GoalCritic>());
        // registerCritic(std::make_unique<critics::ObstaclesCritic>());
        registerCritic(std::make_unique<critics::PathAlignCritic>());
        registerCritic(std::make_unique<critics::PathAlignLegacyCritic>());
        registerCritic(std::make_unique<critics::PathAngleCritic>());
        registerCritic(std::make_unique<critics::PathFollowCritic>());
        registerCritic(std::make_unique<critics::PreferForwardCritic>());
        registerCritic(std::make_unique<critics::TwirlingCritic>());
        registerCritic(std::make_unique<critics::VelocityDeadbandCritic>());

        std::cout<<"[nav2_mppi_controller] Critics registered. Initializing optimizer..."<<std::endl;
        std::cout<<"[nav2_mppi_controller] Number of critics: "<<critics_.size()<<std::endl;

        optimizer_.initialize(
            "MPPIController",
            critics_,
            params
            );
       //TODO:初始化过程需要完善
    }

     std::optional<finenav::DataPacket<OutputProfile>>plan(const finenav::PlanningContext<MPPIController>& ctx, const IMapView& map_view) {

        finenav::DataPacket<OutputProfile> traj;
        // 从框架注入的 RobotState 中取出 pose 和 twist，无需 TF 查询
        const auto& robot_state = ctx.robot_state;
        auto goal_pose = ctx.goal_pose;

        nav_msgs::msg::Path path = convertDataPacketToPath(ctx.ref_traj, "map");
        this->setPlan(path);

        geometry_msgs::msg::PoseStamped start_pose_stamped;
        start_pose_stamped.pose = robot_state.pose;
        start_pose_stamped.header.stamp = rclcpp::Clock().now();
        start_pose_stamped.header.frame_id = "map";

        computeVelocityCommands(
            start_pose_stamped,
            robot_state.twist,  // 直接使用当前实际速度
            map_view
            );
        // 装填输出轨迹

    // 获取最优轨迹
    auto optimized_trajectory = optimizer_.getOptimizedTrajectory();
    auto control_seq = optimizer_.getControlSequence();
    // 填充轨迹点 (pose + twist)
    const size_t n = optimized_trajectory.shape(0);
    traj.poses.reserve(n);
    traj.twists.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        // pose: x, y, yaw -> quaternion
        geometry_msgs::msg::Pose pose;
        pose.position.x = optimized_trajectory(i, 0);
        pose.position.y = optimized_trajectory(i, 1);
        pose.position.z = 0.0;
        const float yaw = optimized_trajectory(i, 2);
        pose.orientation.z = std::sin(yaw * 0.5f);
        pose.orientation.w = std::cos(yaw * 0.5f);
        traj.poses.push_back(pose);

        // twist: vx, vy, wz from control sequence
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = control_seq(i, 0);
        twist.angular.z = control_seq(i, 1);
        twist.linear.y  = control_seq(i, 2);
        traj.twists.push_back(twist);
    }

    return traj;
}


  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage);

  /**
   * @brief Reset optimizer state at Episode boundary (new goal).
   *        Called by ControlLayer whenever a new Action Goal arrives.
   *        Clears control_sequence_ so the next plan() call starts from
   *        a zero warm-start, independent of any previous planning direction.
   */
  void reset() {
      optimizer_.reset();
  }

    void registerCritic (std::unique_ptr<critics::CriticFunction> critic) {
        critics_.push_back(std::move(critic));        //TODO:在初始化的时候要换成注册
    }


    template <typename DataPacketT>
    nav_msgs::msg::Path convertDataPacketToPath(
        const DataPacketT & data,
        const std::string & frame_id)
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // ⭐ 关键：只有当有 poses 才能用
        if constexpr (requires { data.poses; }) {

            path.poses.reserve(data.poses.size());

            for (const auto & pose : data.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose = pose;

                path.poses.push_back(ps);
            }
        }

        return path;
    }


protected:
    /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
//    void visualize(
//        nav_msgs::msg::Path transformed_plan,
//        const builtin_interfaces::msg::Time & cmd_stamp,
//        double z_height);

    /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
	// geometry_msgs::msg::TwistStamped computeVelocityCommands(
    void computeVelocityCommands(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
  		const geometry_msgs::msg::Twist & robot_speed,
    	const IMapView& map_view)
	{
	#ifdef BENCHMARK_TESTING
  	auto start = std::chrono::system_clock::now();
	#endif

	//  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
	//    reset();
	//  }
//  	last_time_called_ = clock_->now();

//  	std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
//

  	// geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, path_ ,map_view);

	#ifdef BENCHMARK_TESTING
  	auto end = std::chrono::system_clock::now();
  	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  	RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
	#endif

//  	if (visualize_) {
//    	double z_height = 0.06;
//    	const std::string base_frame = map_view->getBaseFrameID();
//    	try {
//      	auto tfst = tf_buffer_->lookupTransform(
//        	map_view.getGlobalFrameID(), base_frame,  cmd.header.stamp);
//      		z_height = tfst.transform.translation.z;
//    	} catch (const tf2::TransformException & ex) {
//      	RCLCPP_WARN(logger_, "Failed to get transform %s -> %s: %s",
//                  map_view.getGlobalFrameID().c_str(), base_frame.c_str(), ex.what());
//    	}
//    	visualize(std::move(path_, cmd.header.stamp, z_height);
//  	}
  	// return cmd;
}

//  std::string name_;
//  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
//  rclcpp::Clock::SharedPtr clock_;
//  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
//  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;

//  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
//  TrajectoryVisualizer trajectory_visualizer_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  bool visualize_;

  double reset_period_;
  // Last time computeVelocityCommands was called
//  rclcpp::Time last_time_called_;
};

}  // namespace nav2_mppi_controller


FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)


#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critic_data.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "nav2_core/goal_checker.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "finenav_mppi_controller/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 */
struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;

  xt::xtensor<float, 1> & costs;
  float & model_dt;

  bool fail_flag;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critic_function.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */

class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string & parent_name,
    const std::string & name,
    const finenav_mppi_controller::Params & params)
  {
    parent_ = parent;
    name_ = name;
    parent_name_ = parent_name;

    initialize(params);
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data, const nav2_mppi_controller::IMapView & map_view) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize(const finenav_mppi_controller::Params & params) = 0;

  /**
    * @brief Get name of critic#include "finenav_control_layers/local_layer.hpp"
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("CriticFunction")};

};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critic_manager.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <xtensor/xtensor.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/critic_function.hpp"

#include "finenav_mppi_controller/critics/constraint_critic.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"
#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"
#include "finenav_mppi_controller/critics/goal_critic.hpp"
#include "finenav_mppi_controller/critics/obstacles_critic.hpp"
#include "finenav_mppi_controller/critics/path_align_critic.hpp"
#include "finenav_mppi_controller/critics/path_align_legacy_critic.hpp"
#include "finenav_mppi_controller/critics/path_angle_critic.hpp"
#include "finenav_mppi_controller/critics/path_follow_critic.hpp"
#include "finenav_mppi_controller/critics/prefer_forward_critic.hpp"
#include "finenav_mppi_controller/critics/twirling_critic.hpp"
#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */

class CriticManager
{
public:
  /**
    * @brief Constructor for mppi::CriticManager
    */
  CriticManager() = default;

  /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
  virtual ~CriticManager() = default;

  /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
     std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
     const finenav_mppi_controller::Params & params);

  /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */

  void evalTrajectoriesScores(CriticData & data ,const nav2_mppi_controller::IMapView& map_view) const;

protected:
  /**
    * @brief Get parameters (critics to load)
    */
  void getParams();


  /**
    * @brief Get full-name namespaced critic IDs
    */
  std::string getFullName(const std::string & name);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_MANAGER_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/constraint_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

  void GetWeight() {
    RCLCPP_DEBUG(logger_, "Weight = %f", weight_);
  }

  float getMaxVelConstraint() {return max_vel_;}
  float getMinVelConstraint() {return min_vel_;}

protected:
  unsigned int power_{0};
  float weight_{0};
  float min_vel_;
  float max_vel_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/cost_critic.hpp

```cpp
// Copyright (c) 2023 Robocc Brice Renaudeau
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_

#include <memory>
#include <string>


#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"



namespace mppi::critics
{

/**
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 */

class CostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
    bool inCollision(float x, float y, float theta, const nav2_mppi_controller::IMapView& map_view)
    {
        return map_view.isCollision(x, y, theta);
    }


  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @return Collision information at pose
    */
    float costAtPose(float x, float y, const nav2_mppi_controller::IMapView& map_view)
    {
        // 使用 Position3D 作为参数传入 map_view
        return map_view.getCost(Position3D{static_cast<double>(x),
                                                    static_cast<double>(y),
                                                    0.0});
    }

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
//  float findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);

protected:
  float possibly_inscribed_cost_;

//  bool consider_footprint_{true};
  float circumscribed_radius_{0};
  float circumscribed_cost_{0};
  float collision_cost_{0};
  float critical_cost_{0};
  float weight_{0};

  float near_goal_distance_;
  unsigned int power_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__COST_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/goal_angle_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for driving towards goal orientation
 */

class GoalAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  float threshold_to_consider_{0};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_ANGLE_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/goal_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for driving towards goal
 */

class GoalCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__GOAL_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/obstacles_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_

#include <memory>

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;
/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for avoiding obstacles, allowing it to deviate off
 * the planned path. This is important to tune in tandem with PathAlign to make a balance
 * between path-tracking and dynamic obstacle avoidance capabilities as desirable for a
 * particular application
 */

class ObstaclesCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
    void score(CriticData & data,const nav2_mppi_controller::IMapView& map_view) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Costmap cost
    * @return bool if in collision
    */
    bool inCollision(float cost, const nav2_mppi_controller::IMapView& map_view) const
    {
        bool is_tracking_unknown =
          map_view.isTrackingUnknown();

        switch (static_cast<unsigned char>(cost)) {
            // 使用具体数值替代 nav2_costmap_2d 宏
            // NO_INFORMATION = 255, LETHAL_OBSTACLE = 254, INSCRIBED_INFLATED_OBSTACLE = 253
            case 254: // LETHAL_OBSTACLE
                return true;
            case 253: // INSCRIBED_INFLATED_OBSTACLE
                return consider_footprint_ ? false : true;
            case 255: // NO_INFORMATION
                return is_tracking_unknown ? false : true;
        }

        return false;
    }

  /**
    * @brief cost at a robot pose
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return Collision information at pose
    */
    float costAtPose(float x, float y, float theta, const nav2_mppi_controller::IMapView& map_view)
    {
        float collision_cost;
        collision_cost = map_view.getCost(Position3D{static_cast<double>(x),
                                                  static_cast<double>(y),
                                                  0.0});

        if (consider_footprint_ )
        {
            collision_cost = static_cast<float>(map_view.costAtPose(
                x, y, theta));
        }

        return collision_cost;
    }

  /**
    * @brief Distance to obstacle from cost
    * @param cost Costmap cost
    * @return float Distance to the obstacle represented by cost
    */
  float distanceToObstacle(const float & cost, const IMapView& map_view) const;


  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  // float findCircumscribedCost(const IMapView& map_view);//这个函数仅在有膨胀机制时候有作用

protected:

  bool consider_footprint_{true};
  float collision_cost_{0};
  float inflation_scale_factor_{0}, inflation_radius_{0};

  float min_radius_{0};
  float possibly_inscribed_cost_;
  float collision_margin_distance_;
  float near_goal_distance_;
  float circumscribed_cost_{0}, circumscribed_radius_{0};

  unsigned int power_{0};
  float repulsion_weight_, critical_weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OBSTACLES_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_align_critic.hpp

```cpp
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to the path. Note:
 * High settings of this will follow the path more precisely, but also makes it
 * difficult (or impossible) to deviate in the presence of dynamic obstacles.
 * This is an important critic to tune and consider in tandem with Obstacle.
 */

class PathAlignCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_path_orientations_{false};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_align_legacy_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_LEGACY_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_LEGACY_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"


namespace mppi::critics
{

/**
 * @class mppi::critics::PathAlignLegacyCritic
 * @brief Critic objective function for aligning to the path. Note:
 * High settings of this will follow the path more precisely, but also makes it
 * difficult (or impossible) to deviate in the presence of dynamic obstacles.
 * This is an important critic to tune and consider in tandem with Obstacle.
 * This is the initial 'Legacy' implementation before replacement Oct 2023.
 */

class PathAlignLegacyCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  size_t offset_from_furthest_{0};
  int trajectory_point_step_{0};
  float threshold_to_consider_{0};
  float max_path_occupancy_ratio_{0};
  bool use_path_orientations_{false};
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ALIGN_LEGACY_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_angle_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"


namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to path in cases of extreme misalignment
 * or turning
 */

class PathAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  float max_angle_to_furthest_{0};
  float threshold_to_consider_{0};

  size_t offset_from_furthest_{0};
  bool reversing_allowed_{true};
  bool forward_preference_{true};

  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_ANGLE_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/path_follow_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"


namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for following the path approximately
 * To allow for deviation from path in case of dynamic obstacles. Path Align
 * is what aligns the trajectories to the path more or less precisely, if desireable.
 * A higher weight here with an offset > 1 will accelerate the samples to full speed
 * faster and push the follow point further ahead, creating some shortcutting.
 */

class PathFollowCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  float threshold_to_consider_{0};
  size_t offset_from_furthest_{0};

  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PATH_FOLLOW_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/prefer_forward_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for preferring forward motion
 */

class PreferForwardCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__PREFER_FORWARD_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/twirling_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for penalizing wiggling/twirling
 */

class TwirlingCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  unsigned int power_{0};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__TWIRLING_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/critics/velocity_deadband_critic.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_

#include <vector>

#include "finenav_mppi_controller/critic_function.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"


namespace mppi::critics
{

/**
 * @class mppi::critics::VelocityDeadbandCritic
 * @brief Critic objective function for enforcing feasible constraints
 */

class VelocityDeadbandCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize(const finenav_mppi_controller::Params & params) override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data, const nav2_mppi_controller::IMapView& map_view) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  std::vector<float> deadband_velocities_{0.0f, 0.0f, 0.0f};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__VELOCITY_DEADBAND_CRITIC_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/map_view_def.hpp

```cpp
#ifndef FINENAV_MAP_VIEW_DEF_HPP
#define FINENAV_MAP_VIEW_DEF_HPP

#include <Eigen/Core>

using Position3D = Eigen::Vector3d;
namespace nav2_mppi_controller {
    class IMapView {
    public:
        IMapView() =  default;
        virtual ~IMapView() = default;
        // 是否将未知区域视为可通行
        virtual bool isTrackingUnknown() const = 0;

        virtual bool considerFootprint() const = 0;

        virtual bool isCollision(
          float x, float y, float theta) const = 0;

        virtual float getRadius() const = 0;

        // 获取地图索引处的代价值（根据项目需要可改为返回 unsigned char 等）
        virtual int getCost(const Position3D& pos ) const = 0;

        virtual float costAtPose(float x, float y, float theta) const = 0;

        virtual std::string getBaseFrameID() const = 0;
    };
}

#endif
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/constraints.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float vy;
  float wz;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd
{
  float vx;
  float vy;
  float wz;
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/control_sequence.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Control
 * @brief A set of controls
 */
struct Control
{
  float vx, vy, wz;
};

/**
 * @struct mppi::models::ControlSequence
 * @brief A control sequence over time (e.g. trajectory)
 */
struct ControlSequence
{
  xt::xtensor<float, 1> vx;
  xt::xtensor<float, 1> vy;
  xt::xtensor<float, 1> wz;

  void reset(unsigned int time_steps)
  {
    vx = xt::zeros<float>({time_steps});
    vy = xt::zeros<float>({time_steps});
    wz = xt::zeros<float>({time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/optimizer_settings.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include "finenav_mppi_controller/models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0, 0, 0};
  models::ControlConstraints constraints{0, 0, 0, 0};
  models::SamplingStd sampling_std{0, 0, 0};
  float model_dt{0};
  float temperature{0};
  float gamma{0};
  unsigned int batch_size{0};
  unsigned int time_steps{0};
  unsigned int iteration_count{0};
  bool shift_control_sequence{false};
  size_t retry_attempt_limit{0};
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/path.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Path
 * @brief Path represented as a tensor
 */
struct Path
{
  xt::xtensor<float, 1> x;
  xt::xtensor<float, 1> y;
  xt::xtensor<float, 1> yaws;

  /**
    * @brief Reset path data
    */
  void reset(unsigned int size)
  {
    x = xt::zeros<float>({size});
    y = xt::zeros<float>({size});
    yaws = xt::zeros<float>({size});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/state.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_

#include <xtensor/xtensor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State
{
  xt::xtensor<float, 2> vx;
  xt::xtensor<float, 2> vy;
  xt::xtensor<float, 2> wz;

  xt::xtensor<float, 2> cvx;
  xt::xtensor<float, 2> cvy;
  xt::xtensor<float, 2> cwz;

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;

  /**
    * @brief Reset state data
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = xt::zeros<float>({batch_size, time_steps});
    vy = xt::zeros<float>({batch_size, time_steps});
    wz = xt::zeros<float>({batch_size, time_steps});

    cvx = xt::zeros<float>({batch_size, time_steps});
    cvy = xt::zeros<float>({batch_size, time_steps});
    cwz = xt::zeros<float>({batch_size, time_steps});
  }
};
}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/models/trajectories.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

namespace mppi::models
{

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories
{
  xt::xtensor<float, 2> x;
  xt::xtensor<float, 2> y;
  xt::xtensor<float, 2> yaws;

  /**
    * @brief Reset state data
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = xt::zeros<float>({batch_size, time_steps});
    y = xt::zeros<float>({batch_size, time_steps});
    yaws = xt::zeros<float>({batch_size, time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/motion_models.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>
#include <string>

#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>

//#include "finenav_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    using namespace xt::placeholders;  // NOLINT
    xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
      xt::view(state.cvx, xt::all(), xt::range(0, -1));

    xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
      xt::view(state.cwz, xt::all(), xt::range(0, -1));

    if (isHolonomic()) {
      xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
        xt::view(state.cvy, xt::all(), xt::range(0, -1));
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(const std::string & name)
  {
    min_turning_r_= 0.2 ;
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, (xt::fabs(vx) / xt::fabs(wz)) < min_turning_r_);
    view = xt::sign(wz) * xt::fabs(vx) / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/optimizer.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_

#pragma once

#include <string>
#include <memory>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "finenav_mppi_controller/models/optimizer_settings.hpp"
#include "finenav_mppi_controller/motion_models.hpp"
#include "finenav_mppi_controller/critic_manager.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "finenav_mppi_controller/tools/noise_generator.hpp"
//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"



namespace mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;
/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */


class Optimizer
{
public:
  /**
    * @brief Constructor for mppi::Optimizer
    */
  	Optimizer() = default;

  /**
   * @brief Destructor for mppi::Optimizer
   */
  	~Optimizer() {shutdown();}


  /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
    void initialize(
        const std::string & name,
        std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
        const finenav_mppi_controller::Params & params)
  	{
  	    name_ = name;
  	    critics_ = std::move(critics);

  	    getParams(params);

  		critic_manager_.on_configure(critics_, params);
  	    noise_generator_.initialize(settings_, isHolonomic());

  	    reset();

  		std::cout<<"[nav2_mppi_controller] Optimizing initialized"<<std::endl;
  	}

  /**
   * @brief Shutdown for optimizer at process end
   */
    void shutdown()
  	{
  	    noise_generator_.shutdown();
  	}

    void getParams(const finenav_mppi_controller::Params & p)
  	{
  	    auto & s = settings_;

  	    s.model_dt        = static_cast<float>(p.model_dt);
  	    s.time_steps      = static_cast<unsigned int>(p.time_steps);
  	    s.batch_size      = static_cast<unsigned int>(p.batch_size);
  	    s.iteration_count = static_cast<unsigned int>(p.iteration_count);
  	    s.temperature     = static_cast<float>(p.temperature);
  	    s.gamma           = static_cast<float>(p.gamma);

  	    s.base_constraints.vx_max = p.vx_max;
  	    s.base_constraints.vx_min = p.vx_min;
  	    s.base_constraints.vy     = p.vy_max;
  	    s.base_constraints.wz     = p.wz_max;

  	    s.sampling_std.vx = p.vx_std;
  	    s.sampling_std.vy = p.vy_std;
  	    s.sampling_std.wz = p.wz_std;

  	    s.retry_attempt_limit = 1;

  	    s.constraints = s.base_constraints;
  	    setMotionModel(p.motion_model);

  		std::cout<<"[nav2_mppi_controller] Optimizing param prepared"<<std::endl;
  	    double controller_frequency = 1.0 / static_cast<double>(s.model_dt);
  	    setOffset(controller_frequency);
  	}

  /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
	// geometry_msgs::msg::TwistStamped evalControl(
	void evalControl(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
    	const geometry_msgs::msg::Twist & robot_speed,
    	const nav_msgs::msg::Path & plan,
    	const nav2_mppi_controller::IMapView& map_view)
    {
    prepare(robot_pose, robot_speed, plan);


    do {
      optimize(map_view);
    } while (fallback(critics_data_.fail_flag));
    // utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    auto control = getControlFromSequenceAsTwist(plan.header.stamp, map_view);
    if (settings_.shift_control_sequence) {
      shiftControlSequence();
    }
    // return control;
}


  /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
    models::Trajectories & getGeneratedTrajectories()
  	{
  	    return generated_trajectories_;
  	}

  /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
    /**
     * @brief Get the optimal control sequence (vx, vy, wz) for each time step
     * @return xtensor of shape [time_steps, 3] with columns (vx, wz, vy) — or (vx, wz, 0) for DiffDrive
     */
    xt::xtensor<float, 2> getControlSequence() const
    {
        auto && seq = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3u});
        xt::noalias(xt::view(seq, xt::all(), 0)) = control_sequence_.vx;
        xt::noalias(xt::view(seq, xt::all(), 1)) = control_sequence_.wz;
        if (isHolonomic()) {
            xt::noalias(xt::view(seq, xt::all(), 2)) = control_sequence_.vy;
        } else {
            xt::noalias(xt::view(seq, xt::all(), 2)) = xt::zeros<float>({settings_.time_steps});
        }
        return seq;
    }

    xt::xtensor<float, 2>getOptimizedTrajectory()
    {
        auto && sequence =
          xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
        auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

        xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
        xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

        if (isHolonomic()) {
            xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
        }

        integrateStateVelocities(trajectories, sequence);
        return std::move(trajectories);
    }


  /**
   * @brief Set the maximum speed based on the speed limits callback
   * @param speed_limit Limit of the speed for use
   * @param percentage Whether the speed limit is absolute or relative
   */
    void setSpeedLimit(double speed_limit, bool percentage)
    {
        auto & s = settings_;

        // 使用局部哨兵值表示 "no speed limit"
        // 说明：nav2_costmap_2d::NO_SPEED_LIMIT 在 nav2 的实现中是用于表示“无速度限制”的常量。
        // 为了避免直接依赖外部宏/命名空间，这里使用本地常量 NO_SPEED_LIMIT_LOCAL 作为替代。
        // 可替代策略还包括使用负值（例如 -1）或 std::optional<double> 来表示未设置限制。
        constexpr double NO_SPEED_LIMIT_LOCAL = -1;

        if (speed_limit == NO_SPEED_LIMIT_LOCAL) {
            s.constraints.vx_max = s.base_constraints.vx_max;
            s.constraints.vx_min = s.base_constraints.vx_min;
            s.constraints.vy = s.base_constraints.vy;
            s.constraints.wz = s.base_constraints.wz;
        } else {
            if (percentage) {
                // Speed limit is expressed in % from maximum speed of robot
                double ratio = speed_limit / 100.0;
                s.constraints.vx_max = s.base_constraints.vx_max * ratio;
                s.constraints.vx_min = s.base_constraints.vx_min * ratio;
                s.constraints.vy = s.base_constraints.vy * ratio;
                s.constraints.wz = s.base_constraints.wz * ratio;
            } else {
                // Speed limit is expressed in absolute value
                double ratio = speed_limit / s.base_constraints.vx_max;
                s.constraints.vx_max = s.base_constraints.vx_max * ratio;
                s.constraints.vx_min = s.base_constraints.vx_min * ratio;
                s.constraints.vy = s.base_constraints.vy * ratio;
                s.constraints.wz = s.base_constraints.wz * ratio;
            }
        }
    }


  /**
   * @brief Reset the optimization problem to initial conditions
   */
    void reset()
    {
        state_.reset(settings_.batch_size, settings_.time_steps);
        control_sequence_.reset(settings_.time_steps);
        control_history_[0] = {0.0, 0.0, 0.0};
        control_history_[1] = {0.0, 0.0, 0.0};
        control_history_[2] = {0.0, 0.0, 0.0};
        control_history_[3] = {0.0, 0.0, 0.0};

        settings_.constraints = settings_.base_constraints;

        costs_ = xt::zeros<float>({settings_.batch_size});
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

        noise_generator_.reset(settings_, isHolonomic());
        RCLCPP_INFO(logger_, "Optimizer reset");
    }


protected:
  /**
   * @brief Main function to generate, score, and return trajectories
   */

	void optimize(const nav2_mppi_controller::IMapView& map_view)
	{
  	for (size_t i = 0; i < settings_.iteration_count; ++i) {
    	generateNoisedTrajectories();
    	critic_manager_.evalTrajectoriesScores(critics_data_,map_view);
    	updateControlSequence();
 		}
	}
  /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   */
    void prepare(
        const geometry_msgs::msg::PoseStamped & robot_pose,
        const geometry_msgs::msg::Twist & robot_speed,
        const nav_msgs::msg::Path & plan)
	{
	    state_.pose = robot_pose;
	    state_.speed = robot_speed;
	    path_ = utils::toTensor(plan);
	    costs_.fill(0);

	    critics_data_.fail_flag = false;
	    critics_data_.motion_model = motion_model_;
	    critics_data_.furthest_reached_path_point.reset();
	    critics_data_.path_pts_valid.reset();
	}



  /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
    void setMotionModel(const std::string & model)
    {
        if (model == "DiffDrive") {
            motion_model_ = std::make_shared<DiffDriveMotionModel>();
        } else if (model == "Omni") {
            motion_model_ = std::make_shared<OmniMotionModel>();
        } else if (model == "Ackermann") {
            motion_model_ = std::make_shared<AckermannMotionModel>(name_);
        } else {
            throw std::runtime_error(
                    std::string(
                      "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
                      "or Ackermann"));
        }
    }

  /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
    void shiftControlSequence()
    {
        using namespace xt::placeholders;  // NOLINT
        control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
        control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


        xt::view(control_sequence_.vx, -1) =
          xt::view(control_sequence_.vx, -2);

        xt::view(control_sequence_.wz, -1) =
          xt::view(control_sequence_.wz, -2);


        if (isHolonomic()) {
            control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
            xt::view(control_sequence_.vy, -1) =
              xt::view(control_sequence_.vy, -2);
        }
    }

  /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
    void generateNoisedTrajectories()
    {
        noise_generator_.setNoisedControls(state_, control_sequence_);
        noise_generator_.generateNextNoises();
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
    }

  /**
   * @brief Apply hard vehicle constraints on control sequence
   */
    void applyControlSequenceConstraints()
    {
        auto & s = settings_;

        if (isHolonomic()) {
            control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
        }

        control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
        control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

        motion_model_->applyConstraints(control_sequence_);
    }


  /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
    void updateStateVelocities(
      models::State & state) const
    {
        updateInitialStateVelocities(state);
        propagateStateVelocitiesFromInitials(state);
    }


  /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
    void updateInitialStateVelocities(
      models::State & state) const
    {
        xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
        xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

        if (isHolonomic()) {
            xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
        }
    }
  /**
   * @brief predict velocities in state using model
   * for time horizon equal to timesteps
   * @param state fill state
   */
    void propagateStateVelocitiesFromInitials(
      models::State & state) const
    {
        motion_model_->predict(state);
    }

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
    void integrateStateVelocities(
		xt::xtensor<float, 2> & trajectory,
		const xt::xtensor<float, 2> & sequence) const
    {
        float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

        const auto vx = xt::view(sequence, xt::all(), 0);
        const auto vy = xt::view(sequence, xt::all(), 2);
        const auto wz = xt::view(sequence, xt::all(), 1);

        auto traj_x = xt::view(trajectory, xt::all(), 0);
        auto traj_y = xt::view(trajectory, xt::all(), 1);
        auto traj_yaws = xt::view(trajectory, xt::all(), 2);

        xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

        auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
        auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

        const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

        xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
        xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
        xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
        xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

        auto && dx = xt::eval(vx * yaw_cos);
        auto && dy = xt::eval(vx * yaw_sin);

        if (isHolonomic()) {
            dx = dx - vy * yaw_sin;
            dy = dy + vy * yaw_cos;
        }

        xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
        xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
    }

	void integrateStateVelocities(
		models::Trajectories & trajectories,
		const models::State & state) const
	{
		const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

		xt::noalias(trajectories.yaws) =
            xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;

		const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

		auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
		auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
		xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
		xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
		xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
		xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

		auto && dx = xt::eval(state.vx * yaw_cos);
		auto && dy = xt::eval(state.vx * yaw_sin);

		if (isHolonomic()) {
			dx = dx - state.vy * yaw_sin;
			dy = dy + state.vy * yaw_cos;
		}

		xt::noalias(trajectories.x) = state.pose.pose.position.x +
            xt::cumsum(dx * settings_.model_dt, 1);
		xt::noalias(trajectories.y) = state.pose.pose.position.y +
            xt::cumsum(dy * settings_.model_dt, 1);
	}


  /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
    void updateControlSequence()
    {
        auto & s = settings_;
        auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
        auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
        xt::noalias(costs_) +=
          s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
          xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
        xt::noalias(costs_) +=
          s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
          xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

        if (isHolonomic()) {
            auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
            xt::noalias(costs_) +=
              s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
              xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
              1, immediate);
        }

        auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
        auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
        auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
        auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

        xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
        xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
        if (isHolonomic()) {
            xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
        }

        applyControlSequenceConstraints();
    }

  /**
   * @brief Convert control sequence to a twist commant
   * @param stamp Timestamp to use
   * @return TwistStamped of command to send to robot base
   */
    geometry_msgs::msg::TwistStamped getControlFromSequenceAsTwist(
        const builtin_interfaces::msg::Time & stamp, const nav2_mppi_controller::IMapView& map_view)
    {
        unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

        auto vx = control_sequence_.vx(offset);
        auto wz = control_sequence_.wz(offset);

        if (isHolonomic()) {
            auto vy = control_sequence_.vy(offset);
            return utils::toTwistStamped(vx, vy, wz, stamp, map_view.getBaseFrameID());
        }

        return utils::toTwistStamped(vx, wz, stamp, map_view.getBaseFrameID());
    }
  /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
    bool isHolonomic() const {return motion_model_->isHolonomic();}

  /**
   * @brief Using control frequence and time step size, determine if trajectory
   * offset should be used to populate initial state of the next cycle
   */
    void setOffset(double controller_frequency)
    {
        const double controller_period = 1.0 / controller_frequency;
        constexpr double eps = 1e-6;

        if ((controller_period + eps) < settings_.model_dt) {
            RCLCPP_WARN(
              logger_,
              "Controller period is less then model dt, consider setting it equal");
        } else if (abs(controller_period - settings_.model_dt) < eps) {
            RCLCPP_INFO(
              logger_,
              "Controller period is equal to model dt. Control sequence "
              "shifting is ON");
            settings_.shift_control_sequence = true;
        } else {
            throw std::runtime_error(
                    "Controller period more then model dt, set it equal to model dt");
        }
    }


  /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
    bool fallback(bool fail)
    {
        static size_t counter = 0;

        if (!fail) {
            counter = 0;
            return false;
        }

        reset();

        if (++counter > settings_.retry_attempt_limit) {
            counter = 0;
            throw std::runtime_error("Optimizer fail to compute path");
        }

        return true;
    }

protected:
//  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

//  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  xt::xtensor<float, 1> costs_;

  CriticData critics_data_ =
  {state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/tools/noise_generator.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "finenav_mppi_controller/models/optimizer_settings.hpp"
//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 */
class NoiseGenerator
{
public:
  /**
    * @brief Constructor for mppi::NoiseGenerator
    */
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   * @param name Namespace for configs
   * @param param_handler Get parameters util
   */
  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool is_holonomic);

  /**
   * @brief Shutdown noise generator thread
   */
  void shutdown();

  /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
  void generateNextNoises();

  /**
   * @brief set noised control_sequence to state controls
   * @return noises vx, vy, wz
   */
  void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

  /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
  /**
   * @brief Thread to execute noise generation process
   */
  void noiseThread();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  xt::xtensor<float, 2> noises_vx_;
  xt::xtensor<float, 2> noises_vy_;
  xt::xtensor<float, 2> noises_wz_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false}, regenerate_noises_{false};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/include/finenav_mppi_controller/tools/utils.hpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "finenav_mppi_controller/models/optimizer_settings.hpp"
#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include <Eigen/Core>

using Position3D = Eigen::Vector3d;
namespace mppi::utils
{
using xt::evaluation_strategy::immediate;

/**
 * @brief Convert data into pose
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @return Pose object
 */
inline geometry_msgs::msg::Pose createPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  return pose;
}

/**
 * @brief Convert data into scale
 * @param x X scale
 * @param y Y scale
 * @param z Z scale
 * @return Scale object
 */
inline geometry_msgs::msg::Vector3 createScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;
  scale.x = x;
  scale.y = y;
  scale.z = z;
  return scale;
}

/**
 * @brief Convert data into color
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param a Alpha component (transparency)
 * @return Color object
 */
inline std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

/**
 * @brief Convert data into a Maarker
 * @param id Marker ID
 * @param pose Marker pose
 * @param scale Marker scale
 * @param color Marker color
 * @param frame Reference frame to use
 * @return Visualization Marker
 */
inline visualization_msgs::msg::Marker createMarker(
  int id, const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
  using visualization_msgs::msg::Marker;
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time(0, 0);
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::SPHERE;
  marker.action = Marker::ADD;

  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);
  twist.twist.linear.y = vy;

  return twist;
}

/**
 * @brief Convert path to a tensor
 * @param path Path to convert
 * @return Path tensor
 */
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return result;
}

/**
 * @brief Check if the robot pose is within the Goal Checker's tolerances to goal
 * @param global_checker Pointer to the goal checker
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within goal checker tolerances to the goal
 */
inline bool withinPositionGoalTolerance(
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;

    // 硬编码 pose_tolerance 的值
    pose_tolerance.position.x = 0.1;
    pose_tolerance.position.y = 0.1;
    pose_tolerance.position.z = 0.1;
    pose_tolerance.orientation.x = 0.0;
    pose_tolerance.orientation.y = 0.0;
    pose_tolerance.orientation.z = 0.0;
    pose_tolerance.orientation.w = 1.0;


    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal_x;
    auto dy = robot.position.y - goal_y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }


  return false;
}

/**
 * @brief Check if the robot pose is within tolerance to the goal
 * @param pose_tolerance Pose tolerance to use
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within tolerance to the goal
 */
inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

  auto dx = robot.position.x - goal_x;
  auto dy = robot.position.y - goal_y;

  auto dist_sq = dx * dx + dy * dy;

  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  return false;
}

/**
  * @brief normalize
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */
template<typename T>
auto normalize_angles(const T & angles)
{
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}

/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of furthest path point reached by a set of trajectories
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  const auto dx = data.path.x - traj_x;
  const auto dy = data.path.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0, min_id_by_path = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();
  float cur_dist = 0.0f;

  for (size_t i = 0; i < dists.shape(0); i++) {
    min_id_by_path = 0;
    min_distance_by_path = std::numeric_limits<float>::max();
    for (size_t j = 0; j < dists.shape(1); j++) {
      cur_dist = dists(i, j);
      if (cur_dist < min_distance_by_path) {
        min_distance_by_path = cur_dist;
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}

/**
 * @brief Evaluate closest point idx of data.path which is
 * nearset to the start of the trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of closest path point at start of the trajectories
 */
inline size_t findPathTrajectoryInitialPoint(const CriticData & data)
{
  // First point should be the same for all trajectories from initial conditions
  const auto dx = data.path.x - data.trajectories.x(0, 0);
  const auto dy = data.path.y - data.trajectories.y(0, 0);
  const auto dists = dx * dx + dy * dy;

  float min_distance_by_path = std::numeric_limits<float>::max();
  size_t min_id = 0;
  for (size_t j = 0; j < dists.shape(0); j++) {
    if (dists(j) < min_distance_by_path) {
      min_distance_by_path = dists(j);
      min_id = j;
    }
  }

  return min_id;
}

/**
 * @brief evaluate path furthest point if it is not set
 * @param data Data to use
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief evaluate path costs
 * @param data Data to use
 */
template <typename MapView>
inline void findPathCosts(
  CriticData & data,
  const MapView& map_view)
{
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    const auto path_x = data.path.x(idx);
    const auto path_y = data.path.y(idx);
//    if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {   //TODO：应该做个检查是否在地图内
//      (*data.path_pts_valid)[idx] = false;
//      continue;
//    }

    switch (map_view.getCost(Position3D{static_cast<double>(path_x),                //根据finenav—engine的要求，坐标均为全局坐标，即fixed frame坐标系下的坐标
                                                    static_cast<double>(path_y),
                                                    0.0})) {
      // 使用具体数值替代 nav2_costmap_2d 宏常量：
      // NO_INFORMATION = 255, LETHAL_OBSTACLE = 254, INSCRIBED_INFLATED_OBSTACLE = 253
      case 254: // LETHAL_OBSTACLE
        (*data.path_pts_valid)[idx] = false;
        continue;
      case 253: // INSCRIBED_INFLATED_OBSTACLE
        (*data.path_pts_valid)[idx] = false;
        continue;
      case 255: // NO_INFORMATION
      {
        const bool is_tracking_unknown =
          map_view.isTrackingUnknown();
        (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
        continue;
      }
    }

    (*data.path_pts_valid)[idx] = true;
  }
}

/**
 * @brief evaluate path costs if it is not set
 * @param data Data to use
 */
template <typename MapView>
inline void setPathCostsIfNotSet(
  CriticData & data,
  const MapView& map_view)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, map_view);
  }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param forward_preference If reversing direction is valid
 * @return Angle between two points
 */
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation);

  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  // If no preference for forward, return smallest angle either in heading or 180 of heading
  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PI))));
  }

  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Apply Savisky-Golay filter to optimal trajectory
 * @param control_sequence Sequence to apply filter to
 * @param control_history Recent set of controls for edge-case handling
 * @param Settings Settings to use
 */
inline void savitskyGolayFilter(
  models::ControlSequence & control_sequence,
  std::array<mppi::models::Control, 4> & control_history,
  const models::OptimizerSettings & settings)
{
  // Savitzky-Golay Quadratic, 9-point Coefficients
  xt::xarray<float> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
  filter /= 231.0;

  const unsigned int num_sequences = control_sequence.vx.shape(0) - 1;

  // Too short to smooth meaningfully
  if (num_sequences < 20) {
    return;
  }

  auto applyFilter = [&](const xt::xarray<float> & data) -> float {
      return xt::sum(data * filter, {0}, immediate)();
    };

  auto applyFilterOverAxis =
    [&](xt::xtensor<float, 1> & sequence,
      const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
      unsigned int idx = 0;
      sequence(idx) = applyFilter(
      {
        hist_0,
        hist_1,
        hist_2,
        hist_3,
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_1,
        hist_2,
        hist_3,
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_2,
        hist_3,
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      idx++;
      sequence(idx) = applyFilter(
      {
        hist_3,
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 4)});

      for (idx = 4; idx != num_sequences - 4; idx++) {
        sequence(idx) = applyFilter(
        {
          sequence(idx - 4),
          sequence(idx - 3),
          sequence(idx - 2),
          sequence(idx - 1),
          sequence(idx),
          sequence(idx + 1),
          sequence(idx + 2),
          sequence(idx + 3),
          sequence(idx + 4)});
      }

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 3),
        sequence(idx + 3)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 2),
        sequence(idx + 2),
        sequence(idx + 2)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1),
        sequence(idx + 1)});

      idx++;
      sequence(idx) = applyFilter(
      {
        sequence(idx - 4),
        sequence(idx - 3),
        sequence(idx - 2),
        sequence(idx - 1),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx),
        sequence(idx)});
    };

  // Filter trajectories
  applyFilterOverAxis(
    control_sequence.vx, control_history[0].vx,
    control_history[1].vx, control_history[2].vx, control_history[3].vx);
  applyFilterOverAxis(
    control_sequence.vy, control_history[0].vy,
    control_history[1].vy, control_history[2].vy, control_history[3].vy);
  applyFilterOverAxis(
    control_sequence.wz, control_history[0].wz,
    control_history[1].wz, control_history[2].wz, control_history[3].wz);

  // Update control history
  unsigned int offset = settings.shift_control_sequence ? 1 : 0;
  control_history[0] = control_history[1];
  control_history[1] = control_history[2];
  control_history[2] = control_history[3];
  control_history[3] = {
    control_sequence.vx(offset),
    control_sequence.vy(offset),
    control_sequence.wz(offset)};
}

/**
 * @brief Find the iterator of the first pose at which there is an inversion on the path,
 * @param path to check for inversion
 * @return the first point after the inversion found in the path
 */
inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
{
  // At least 3 poses for a possible inversion
  if (path.poses.size() < 3) {
    return path.poses.size();
  }

  // Iterating through the path to determine the position of the path inversion
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    float oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    float oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    float ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    float ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      return idx + 1;
    }
  }

  return path.poses.size();
}

/**
 * @brief Find and remove poses after the first inversion in the path
 * @param path to check for inversion
 * @return The location of the inversion, return 0 if none exist
 */
inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cropped_path = path;
  const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
  if (first_after_inversion == path.poses.size()) {
    return 0u;
  }

  cropped_path.poses.erase(
    cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
  path = cropped_path;
  return first_after_inversion;
}

/**
 * @brief Compare to trajectory points to find closest path point along integrated distances
 * @param vec Vect to check
 * @return dist Distance to look for
 */
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
  auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);
  if (iter == vec.begin() + init) {
    return 0;
  }
  if (dist - *(iter - 1) < *iter - dist) {
    return iter - 1 - vec.begin();
  }
  return iter - vec.begin();
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
```

---

### 📄 finenav_planner/finenav_mppi_controller/mppic.xml

```xml
<class_libraries>
  <library path="mppi_controller">
    <class type="nav2_mppi_controller::MPPIController" base_class_type="nav2_core::Controller">
      <description>MPPI Controller for Nav2</description>
    </class>
  </library>
</class_libraries>
```

---

### 📄 finenav_planner/finenav_mppi_controller/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>finenav_mppi_controller</name>
  <version>1.1.19</version>
  <description>finenav_mppi_controller</description>
  <maintainer email="stevenmacenski@gmail.com">Steve Macenski</maintainer>
  <maintainer email="budyakov.aleksei@gmail.com">Aleksei Budyakov</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_ros</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_eigen</depend>
  <depend>tf2_ros</depend>
  <depend>std_msgs</depend>
  <depend>xtensor</depend>
  <depend>libomp-dev</depend>
  <depend>benchmark</depend>
  <depend>xsimd</depend>
  <depend>angles</depend>
  <depend>opencv4</depend>

  <depend>finenav_interface</depend>
  <depend>finenav_util</depend>
  <depend>finenav_core</depend>
  <depend>generate_parameter_library</depend>
  <depend>finenav_msgs</depend>
  <depend>finenav_collision_model</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
<!--    <nav2_core plugin="${prefix}/mppic.xml" />-->
<!--    <nav2_mppi_controller plugin="${prefix}/critics.xml" />-->
  </export>

</package>
```

---

### 📄 finenav_planner/finenav_mppi_controller/src/controller.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <chrono>
#include <tf2/exceptions.h>
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

//template <typename MapView>
//void MPPIController<MapView>::visualize(
//  nav_msgs::msg::Path transformed_plan,
//  const builtin_interfaces::msg::Time & cmd_stamp,
//  double z_height)
//{
//  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories", z_height);
//  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory", cmd_stamp, z_height);
//  trajectory_visualizer_.visualize(std::move(transformed_plan));
//}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
    path_ = path;
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

//FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)
//FINENAV_REGISTER_PLANNER_TRAITS(nav2_mppi_controller::MPPIController,
//    finenav::TraitMask::NONE,
//    finenav::TraitMask::POSE)
}  // namespace nav2_mppi_controller

//#include "pluginlib/class_list_macros.hpp"
//PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critic_manager.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critic_manager.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"

namespace mppi
{
using namespace nav2_mppi_controller;

void CriticManager::on_configure(
    std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
    const finenav_mppi_controller::Params & params)
{
    critics_ = std::move(critics);
    RCLCPP_DEBUG(rclcpp::get_logger("Critic Manager"), "Critic Manager configured;");
    for (size_t q = 0; q < critics_.size(); q++) {
        critics_[q]->initialize(params);
    }
}

//void CriticManager::getParams()
//{
//  auto node = parent_.lock();
//  auto getParam = parameters_handler_->getParamGetter(name_);
//  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
//}


std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data, const IMapView& map_view) const
{
    RCLCPP_DEBUG(rclcpp::get_logger("Critic Manager"), "Scoring...");
    for (size_t q = 0; q < critics_.size(); q++) {
        if (data.fail_flag) {
            break;
        }
        critics_[q]->score(data,map_view);
    }
}

}  // namespace mppi
```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/constraint_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/constraint_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void ConstraintCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.ConstraintCritic;
  enabled_ = c.enabled;
  power_   = static_cast<unsigned int>(c.cost_power);
  weight_  = static_cast<float>(c.cost_weight);

  // Velocity limits come from the top-level optimizer params
  const float vx_max = static_cast<float>(p.vx_max);
  const float vy_max = static_cast<float>(p.vy_max);
  const float vx_min = static_cast<float>(p.vx_min);

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

  RCLCPP_DEBUG(logger_, "Critic initialized");
}

void ConstraintCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }
  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
  auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);
      RCLCPP_DEBUG(logger_, "[ConstraintCritic]: Scored");

    return;
  }

  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
    RCLCPP_DEBUG(logger_, "[ConstraintCritic]: Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/cost_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include "finenav_mppi_controller/critics/cost_critic.hpp"


namespace mppi::critics
{
using namespace nav2_mppi_controller;

void CostCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.CostCritic;
  enabled_           = c.enabled;
  power_             = static_cast<unsigned int>(c.cost_power);
  weight_            = static_cast<float>(c.cost_weight);
  critical_cost_     = static_cast<float>(c.critical_cost);
  collision_cost_    = static_cast<float>(c.collision_cost);
  near_goal_distance_= static_cast<float>(c.near_goal_distance);

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;
}



void CostCritic::score(CriticData & data,const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }


  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0);

  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    const auto & traj = data.trajectories;
    float pose_cost;

    for (size_t j = 0; j < traj_len; j++) {
      // The costAtPose doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), map_view);
      if (pose_cost < 1.0f) {continue;}  // In free space

      if (inCollision(traj.x(i, j), traj.y(i, j), traj.yaws(i, j), map_view)) {
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      if (pose_cost >= 253) {
        repulsive_cost[i] += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        repulsive_cost[i] += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    } else {
      repulsive_cost[i] = std::numeric_limits<float>::max();
    }
  }

  data.costs += xt::pow((weight_ * repulsive_cost / traj_len), power_);
  data.fail_flag = all_trajectories_collide;
  RCLCPP_DEBUG(logger_, "Scored");
}


}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/goal_angle_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void GoalAngleCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.GoalAngleCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
}

void GoalAngleCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_ )    //TODO:旧有的检查与距离的逻辑会触发，为什么
  {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;
  const float goal_yaw = data.path.yaws(goal_idx);

  data.costs += xt::pow(
    xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
    weight_, power_);
  RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/goal_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/goal_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

using xt::evaluation_strategy::immediate;

void GoalCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.GoalCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
}

void GoalCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_ )
  {
    return;
  }

  const auto goal_idx = data.path.x.shape(0) - 1;

  const auto goal_x = data.path.x(goal_idx);
  const auto goal_y = data.path.y(goal_idx);

  const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());

  auto dists = xt::sqrt(
    xt::pow(traj_x - goal_x, 2) +
    xt::pow(traj_y - goal_y, 2));

  data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/obstacles_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include "finenav_mppi_controller/critics/obstacles_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void ObstaclesCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.ObstaclesCritic;
  enabled_            = c.enabled;
  power_              = static_cast<unsigned int>(c.cost_power);
  repulsion_weight_   = static_cast<float>(c.repulsion_weight);
  critical_weight_    = static_cast<float>(c.critical_weight);
  consider_footprint_ = c.consider_footprint;
  collision_cost_     = static_cast<float>(c.collision_cost);
  collision_margin_distance_ = static_cast<float>(c.collision_margin_distance);
  near_goal_distance_ = static_cast<float>(c.near_goal_distance);
  inflation_scale_factor_ = static_cast<float>(c.cost_scaling_factor);
  inflation_radius_   = static_cast<float>(c.inflation_radius);

  RCLCPP_DEBUG(logger_, "Critic initialized");
}

//float ObstaclesCritic::findCircumscribedCost(
//  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
//{
//  double result = -1.0;
//  bool inflation_layer_found = false;
//
//  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
//  if (static_cast<float>(circum_radius) == ci<MapView>rcumscribed_radius_) {
//    // early return if footprint size is unchanged
//    return circumscribed_cost_;
//  }
//
//  // check if the costmap has an inflation layer
//  for (auto layer = costmap->getLayeredCostmap()->getPlugins()->begin();
//    layer != costmap->getLayeredCostmap()->getPlugins()->end();
//    ++layer)
//  {
//    auto inflation_layer = std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
//    if (!inflation_layer) {
//      continue;
//    }
//
//    inflation_layer_found = true;
//    const double resolution = costmap->getCostmap()->getResolution();
//    result = inflation_layer->computeCost(circum_radius / resolution);
//    auto getParam = parameters_handler_->getParamGetter(name_);
//    getParam(inflation_scale_factor_, "cost_scaling_factor", 10.0);
//    getParam(inflation_radius_, "inflation_radius", 0.55);
//  }
//
//  if (!inflation_layer_found) {
//    RCLCPP_WARN(
//      logger_,
//      "No inflation layer found in costmap configuration. "
//      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
//      "field to speed up collision checking by only checking the full footprint "
//      "when robot is within possibly-inscribed radius of an obstacle. This may "
//      "significantly slow down planning times and not avoid anything but absolute collisions!");
//  }
//
//  circumscribed_radius_ = static_cast<float>(circum_radius);
//  circumscribed_cost_ = static_cast<float>(result);
//
//  return circumscribed_cost_;
//}

float ObstaclesCritic::distanceToObstacle(const float & cost, const IMapView& map_view) const
{
  const float scale_factor = inflation_scale_factor_;
  const float min_radius = map_view.getRadius();
  float dist_to_obj = (scale_factor * min_radius - log(cost) + log(253.0f)) / scale_factor;

  // If not footprint collision checking, the cost is using the center point cost and
  // needs the radius subtracted to obtain the closest distance to the object

  return dist_to_obj;
}


void ObstaclesCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }

//  if (consider_footprint_) {
//    // footprint may have changed since initialization if user has dynamic footprints
//    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
//  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
  }

  auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  raw_cost.fill(0.0f);
  auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  repulsive_cost.fill(0.0f);

  const size_t traj_len = data.trajectories.x.shape(1);
  bool all_trajectories_collide = true;
  for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
      bool trajectory_collide = false;
      float traj_cost = 0.0f;
      const auto & traj = data.trajectories;
      float pose_cost;

      for (size_t j = 0; j < traj_len; j++) {
          pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j),map_view);
          if (pose_cost < 1.0f) {continue;}  // In free space

          if (inCollision(pose_cost, map_view)) {
              trajectory_collide = true;
              break;
          }

          //   // Cannot process repulsion if inflation layer does not exist
          // if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
          //   continue;
          // }
          const float dist_to_obj = distanceToObstacle(pose_cost, map_view);

          // Let near-collision trajectory points be punished severely
          if (dist_to_obj < collision_margin_distance_) {
              traj_cost += (collision_margin_distance_ - dist_to_obj);
          } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
              repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
          }
      }

      if (!trajectory_collide) {
          all_trajectories_collide = false;
      }
      raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
  }
  data.costs += xt::pow(
    (critical_weight_ * raw_cost) +
    (repulsion_weight_ * repulsive_cost / traj_len),
    power_);
  data.fail_flag = all_trajectories_collide;
}
/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */


}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/path_align_critic.cpp

```cpp
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/path_align_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{
using namespace nav2_mppi_controller;
using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathAlignCritic;
  enabled_                  = c.enabled;
  power_                    = static_cast<unsigned int>(c.cost_power);
  weight_                   = static_cast<float>(c.cost_weight);
  max_path_occupancy_ratio_ = static_cast<float>(c.max_path_occupancy_ratio);
  offset_from_furthest_     = static_cast<size_t>(c.offset_from_furthest);
  trajectory_point_step_    = c.trajectory_point_step;
  threshold_to_consider_    = static_cast<float>(c.threshold_to_consider);
  use_path_orientations_    = c.use_path_orientations;
}

void PathAlignCritic::score(CriticData & data, const IMapView& map_view)
{
  // Don't apply close to goal, let the goal critics take over

  if (!this->enabled_ )
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_segments_count = *data.furthest_reached_path_point;  // up to furthest only
  if (path_segments_count < offset_from_furthest_) {
      RCLCPP_DEBUG(logger_, "Warning");
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  utils::setPathCostsIfNotSet(data, map_view);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  // Find integrated distance in the path
  std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
  float dx = 0.0f, dy = 0.0f;
  for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
  }

  float traj_integrated_distance = 0.0f;
  float summed_path_dist = 0.0f, dyaw = 0.0f;
  float num_samples = 0.0f;
  float Tx = 0.0f, Ty = 0.0f;
  size_t path_pt = 0;
  for (size_t t = 0; t < batch_size; ++t) {
    traj_integrated_distance = 0.0f;
    summed_path_dist = 0.0f;
    num_samples = 0.0f;
    path_pt = 0u;
    const auto T_x = xt::view(data.trajectories.x, t, xt::all());
    const auto T_y = xt::view(data.trajectories.y, t, xt::all());
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      Tx = T_x(p);
      Ty = T_y(p);
      dx = Tx - T_x(p - trajectory_point_step_);
      dy = Ty - T_y(p - trajectory_point_step_);
      traj_integrated_distance += sqrtf(dx * dx + dy * dy);
      path_pt = utils::findClosestPathPt(
        path_integrated_distances, traj_integrated_distance, path_pt);

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynamic obstacles
      if ((*data.path_pts_valid)[path_pt]) {
        dx = P_x(path_pt) - Tx;
        dy = P_y(path_pt) - Ty;
        num_samples += 1.0f;
        if (use_path_orientations_) {
          const auto T_yaw = xt::view(data.trajectories.yaws, t, xt::all());
          dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
          summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
        } else {
          summed_path_dist += sqrtf(dx * dx + dy * dy);
        }
      }
    }
    if (num_samples > 0) {
      cost[t] = summed_path_dist / num_samples;
    } else {
      cost[t] = 0.0f;
    }
  }

  data.costs += xt::pow(std::move(cost) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/path_align_legacy_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/path_align_legacy_critic.hpp"

#include <xtensor/xfixed.hpp>
#include <xtensor/xmath.hpp>

namespace mppi::critics
{
using namespace nav2_mppi_controller;
using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void PathAlignLegacyCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathAlignLegacyCritic;
  enabled_                  = c.enabled;
  power_                    = static_cast<unsigned int>(c.cost_power);
  weight_                   = static_cast<float>(c.cost_weight);
  max_path_occupancy_ratio_ = static_cast<float>(c.max_path_occupancy_ratio);
  offset_from_furthest_     = static_cast<size_t>(c.offset_from_furthest);
  trajectory_point_step_    = c.trajectory_point_step;
  threshold_to_consider_    = static_cast<float>(c.threshold_to_consider);
  use_path_orientations_    = c.use_path_orientations;
}

void PathAlignLegacyCritic::score(CriticData & data, const IMapView& map_view)
{
  // Don't apply close to goal, let the goal critics take over
  if (!this->enabled_ )
  {
    return;
  }

  // Don't apply when first getting bearing w.r.t. the path
  utils::setPathFurthestPointIfNotSet(data);
  if (*data.furthest_reached_path_point < offset_from_furthest_) {
    return;
  }

  // Don't apply when dynamic obstacles are blocking significant proportions of the local path
  utils::setPathCostsIfNotSet(data, map_view);
  const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
  unsigned int invalid_ctr = 0;
  const float range = *data.furthest_reached_path_point - closest_initial_path_point;
  for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
      return;
    }
  }

  const auto & T_x = data.trajectories.x;
  const auto & T_y = data.trajectories.y;
  const auto & T_yaw = data.trajectories.yaws;

  const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
  const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
  const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

  const size_t batch_size = T_x.shape(0);
  const size_t time_steps = T_x.shape(1);
  const size_t traj_pts_eval = floor(time_steps / trajectory_point_step_);
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

  if (path_segments_count < 1) {
    return;
  }

  float dist_sq = 0.0f, dx = 0.0f, dy = 0.0f, dyaw = 0.0f, summed_dist = 0.0f;
  float min_dist_sq = std::numeric_limits<float>::max();
  size_t min_s = 0;

  for (size_t t = 0; t < batch_size; ++t) {
    summed_dist = 0.0f;
    for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
      min_dist_sq = std::numeric_limits<float>::max();
      min_s = 0;

      // Find closest path segment to the trajectory point<MapView>
      for (size_t s = 0; s < path_segments_count - 1; s++) {
        xt::xtensor_fixed<float, xt::xshape<2>> P;
        dx = P_x(s) - T_x(t, p);
        dy = P_y(s) - T_y(t, p);
        if (use_path_orientations_) {
          dyaw = angles::shortest_angular_distance(P_yaw(s), T_yaw(t, p));
          dist_sq = dx * dx + dy * dy + dyaw * dyaw;
        } else {
          dist_sq = dx * dx + dy * dy;
        }
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          min_s = s;
        }
      }

      // The nearest path point to align to needs to be not in collision, else
      // let the obstacle critic take over in this region due to dynam<MapView>ic obstacles
      if (min_s != 0 && (*data.path_pts_valid)[min_s]) {
        summed_dist += sqrtf(min_dist_sq);
      }
    }

    cost[t] = summed_dist / traj_pts_eval;
  }

  data.costs += xt::pow(std::move(cost) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/path_angle_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/path_angle_critic.hpp"

#include <math.h>

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void PathAngleCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathAngleCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  offset_from_furthest_  = static_cast<size_t>(c.offset_from_furthest);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
  max_angle_to_furthest_ = static_cast<float>(c.max_angle_to_furthest);

  // Determine reversing capability from top-level vx_min
  const float vx_min = static_cast<float>(p.vx_min);
  reversing_allowed_  = (vx_min < 0.0f);
  forward_preference_ = !reversing_allowed_;
}

void PathAngleCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);

  if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  auto yaws =
    xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  if (reversing_allowed_ && !forward_preference_) {
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
    data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
      RCLCPP_DEBUG(logger_, "Scored");
  } else {
    data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
      RCLCPP_DEBUG(logger_, "Scored");
  }
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/path_follow_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/path_follow_critic.hpp"

#include <xtensor/xmath.hpp>
#include <xtensor/xsort.hpp>

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void PathFollowCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PathFollowCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  offset_from_furthest_  = static_cast<size_t>(c.offset_from_furthest);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
}

void PathFollowCritic::score(CriticData & data, const IMapView& map_view)
{
  if (!this->enabled_ || data.path.x.shape(0) < 2 ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }
  utils::setPathFurthestPointIfNotSet(data);
  utils::setPathCostsIfNotSet(data, map_view);
  const size_t path_size = data.path.x.shape(0) - 1;

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size-1);
  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  bool valid = false;
  while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
      offseted_idx++;
    }
  }

  const auto path_x = data.path.x(offseted_idx);
  const auto path_y = data.path.y(offseted_idx);

  const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

  auto dists = xt::sqrt(
    xt::pow(last_x - path_x, 2) +
    xt::pow(last_y - path_y, 2));

  data.costs += xt::pow(weight_ * std::move(dists), power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/prefer_forward_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/prefer_forward_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void PreferForwardCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.PreferForwardCritic;
  enabled_               = c.enabled;
  power_                 = static_cast<unsigned int>(c.cost_power);
  weight_                = static_cast<float>(c.cost_weight);
  threshold_to_consider_ = static_cast<float>(c.threshold_to_consider);
}

void PreferForwardCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_ )
  {
    return;
  }

  auto backward_motion = xt::maximum(-data.state.vx, 0);
  data.costs += xt::pow(
    xt::sum(
      std::move(
        backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/twirling_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/twirling_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void TwirlingCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.TwirlingCritic;
  enabled_ = c.enabled;
  power_   = static_cast<unsigned int>(c.cost_power);
  weight_  = static_cast<float>(c.cost_weight);
}


void TwirlingCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;
  if (!this->enabled_)
  {
    return;
  }

  const auto wz = xt::abs(data.state.wz);
  data.costs += xt::pow(xt::mean(wz, {1}, immediate) * weight_, power_);
    RCLCPP_DEBUG(logger_, "Scored");
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/critics/velocity_deadband_critic.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{
using namespace nav2_mppi_controller;

void VelocityDeadbandCritic::initialize(const finenav_mppi_controller::Params & p)
{
  const auto & c = p.VelocityDeadbandCritic;
  enabled_ = c.enabled;
  power_   = static_cast<unsigned int>(c.cost_power);
  weight_  = static_cast<float>(c.cost_weight);

  deadband_velocities_.resize(3);
  const auto & dv = c.deadband_velocities;
  for (size_t i = 0; i < 3 && i < dv.size(); ++i) {
    deadband_velocities_[i] = static_cast<float>(dv[i]);
  }
//
//  RCLCPP_INFO_STREAM(
//    logger_, "VelocityDeadbandCritic instantiated with "
//      << power_ << " power, " << weight_ << " weight, deadband_velocity ["
//      << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
//      << deadband_velocities_.at(2) << "]");
}


void VelocityDeadbandCritic::score(CriticData & data, const IMapView& map_view)
{
  using xt::evaluation_strategy::immediate;

  if (!this->enabled_) {
    return;
  }

  auto & vx = data.state.vx;
  auto & wz = data.state.wz;

  if (data.motion_model->isHolonomic()) {
    auto & vy = data.state.vy;
    if (power_ > 1u) {
      data.costs += xt::pow(
        xt::sum(
          std::move(
            xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
            xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
            xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
          data.model_dt,
          {1}, immediate) *
        weight_,
        power_);
    } else {
      data.costs += xt::sum(
        (std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(1)) - xt::fabs(vy), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
        data.model_dt,
        {1}, immediate) *
        weight_;
    }
    return;
  }

  if (power_ > 1u) {
    data.costs += xt::pow(
      xt::sum(
        std::move(
          xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
          xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0)) *
        data.model_dt,
        {1}, immediate) *
      weight_,
      power_);
  } else {
    data.costs += xt::sum(
      (std::move(
        xt::maximum(fabs(deadband_velocities_.at(0)) - xt::fabs(vx), 0) +
        xt::maximum(fabs(deadband_velocities_.at(2)) - xt::fabs(wz), 0))) *
      data.model_dt,
      {1}, immediate) *
      weight_;
  }
  return;
}

}  // namespace mppi::critics

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/finenav_mppi_controller.yaml

```yaml
finenav_mppi_controller:
  # ── Optimizer settings ─────────────────────────────────────────────────────
  time_steps:
    type: int
    default_value: 56
    description: "Number of time steps in trajectory rollout"
    validation:
      gt<>: [0]
  model_dt:
    type: double
    default_value: 0.05
    description: "Time step duration (s). controller_period = model_dt is required."
    validation:
      gt<>: [0.0]
  batch_size:
    type: int
    default_value: 2000
    description: "Number of sampled trajectories per iteration"
    validation:
      gt<>: [0]
  iteration_count:
    type: int
    default_value: 1
    description: "Number of optimization iterations per control cycle"
    validation:
      gt<>: [0]
  temperature:
    type: double
    default_value: 0.3
    description: "MPPI temperature: higher = more exploration, lower = more exploitation"
    validation:
      gt<>: [0.0]
  gamma:
    type: double
    default_value: 0.015
    description: "Control cost regularization coefficient"
  motion_model:
    type: string
    default_value: "DiffDrive"
    description: "Motion model type: DiffDrive, Omni, Ackermann"
    validation:
      one_of<>: [["DiffDrive", "Omni", "Ackermann"]]

  # ── Velocity constraints ───────────────────────────────────────────────────
  vx_max:
    type: double
    default_value: 0.5
    description: "Maximum forward velocity (m/s)"
  vx_min:
    type: double
    default_value: -0.35
    description: "Minimum forward velocity (m/s). Negative = allow reverse."
  vy_max:
    type: double
    default_value: 0.0
    description: "Maximum lateral velocity (m/s). 0 for non-holonomic."
  wz_max:
    type: double
    default_value: 1.9
    description: "Maximum angular velocity (rad/s)"

  # ── Sampling standard deviations ──────────────────────────────────────────
  vx_std:
    type: double
    default_value: 0.2
    description: "Sampling standard deviation for forward velocity"
    validation:
      gt<>: [0.0]
  vy_std:
    type: double
    default_value: 0.2
    description: "Sampling standard deviation for lateral velocity"
    validation:
      gt<>: [0.0]
  wz_std:
    type: double
    default_value: 0.4
    description: "Sampling standard deviation for angular velocity"
    validation:
      gt<>: [0.0]

  # ── Critics ────────────────────────────────────────────────────────────────
  ConstraintCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable ConstraintCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 4.0
      description: "Weight for constraint violation cost"

  GoalCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable GoalCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 5.0
      description: "Weight for goal attraction cost"
    threshold_to_consider:
      type: double
      default_value: 1.4
      description: "Distance from goal at which critic activates (m)"

  GoalAngleCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable GoalAngleCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 3.0
      description: "Weight for goal heading alignment cost"
    threshold_to_consider:
      type: double
      default_value: 0.5
      description: "Distance from goal at which critic activates (m)"

  PreferForwardCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable PreferForwardCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 5.0
      description: "Weight for preferring forward motion"
    threshold_to_consider:
      type: double
      default_value: 0.5
      description: "Distance threshold to activate this critic (m)"

  CostCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable CostCritic (uses costmap inflation cost)"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 3.81
      description: "Weight for costmap cost (internally normalized by 254)"
    critical_cost:
      type: double
      default_value: 300.0
      description: "Penalty added when pose cost >= 253 (near-lethal)"
    collision_cost:
      type: double
      default_value: 1000000.0
      description: "Penalty for trajectories in collision"
    near_goal_distance:
      type: double
      default_value: 1.0
      description: "Distance from goal where repulsion is disabled (m)"
    consider_footprint:
      type: bool
      default_value: true
      description: "Use full footprint collision check instead of center point"

  PathAlignCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable PathAlignCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 14.0
      description: "Weight for path alignment cost"
    max_path_occupancy_ratio:
      type: double
      default_value: 0.05
      description: "Max fraction of path points that may be occupied before disabling critic"
    trajectory_point_step:
      type: int
      default_value: 4
      description: "Step size between trajectory points evaluated for alignment"
      validation:
        gt<>: [0]
    threshold_to_consider:
      type: double
      default_value: 0.5
      description: "Distance to goal below which critic deactivates (m)"
    offset_from_furthest:
      type: int
      default_value: 20
      description: "Offset from furthest reachable path point for alignment target"
    use_path_orientations:
      type: bool
      default_value: false
      description: "Include path point orientation in alignment cost"

  PathAlignLegacyCritic:
    enabled:
      type: bool
      default_value: false
      description: "Enable PathAlignLegacyCritic (legacy variant, prefer PathAlignCritic)"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 10.0
      description: "Weight for path alignment cost"
    max_path_occupancy_ratio:
      type: double
      default_value: 0.07
      description: "Max fraction of path points that may be occupied"
    trajectory_point_step:
      type: int
      default_value: 4
      description: "Step size between trajectory points"
      validation:
        gt<>: [0]
    threshold_to_consider:
      type: double
      default_value: 0.5
      description: "Distance to goal below which critic deactivates (m)"
    offset_from_furthest:
      type: int
      default_value: 20
      description: "Offset from furthest reachable path point"
    use_path_orientations:
      type: bool
      default_value: false
      description: "Include path point orientation in alignment cost"

  PathFollowCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable PathFollowCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 5.0
      description: "Weight for path following cost"
    offset_from_furthest:
      type: int
      default_value: 6
      description: "Offset from furthest reachable path point for follow target"
    threshold_to_consider:
      type: double
      default_value: 1.4
      description: "Distance to goal below which critic deactivates (m)"

  PathAngleCritic:
    enabled:
      type: bool
      default_value: true
      description: "Enable PathAngleCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 2.0
      description: "Weight for path angle cost"
    offset_from_furthest:
      type: int
      default_value: 4
      description: "Offset from furthest reachable path point"
    threshold_to_consider:
      type: double
      default_value: 0.5
      description: "Distance to goal below which critic deactivates (m)"
    max_angle_to_furthest:
      type: double
      default_value: 1.0
      description: "Maximum allowed yaw error to furthest path point (rad)"

  ObstaclesCritic:
    enabled:
      type: bool
      default_value: false
      description: "Enable ObstaclesCritic. Disabled by default; use CostCritic instead."
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    repulsion_weight:
      type: double
      default_value: 1.5
      description: "Weight for obstacle repulsion cost"
    critical_weight:
      type: double
      default_value: 20.0
      description: "Weight for near-collision penalty"
    consider_footprint:
      type: bool
      default_value: false
      description: "Use full footprint collision checking"
    collision_cost:
      type: double
      default_value: 10000.0
      description: "Penalty for trajectories in collision"
    collision_margin_distance:
      type: double
      default_value: 0.1
      description: "Minimum clearance before collision penalty begins (m)"
    near_goal_distance:
      type: double
      default_value: 0.5
      description: "Distance from goal where repulsion is disabled (m)"
    inflation_radius:
      type: double
      default_value: 2.0
      description: "Inflation layer radius (m)"
    cost_scaling_factor:
      type: double
      default_value: 10.0
      description: "Inflation cost scaling factor (matches costmap inflation layer)"

  TwirlingCritic:
    enabled:
      type: bool
      default_value: false
      description: "Enable TwirlingCritic (penalizes unnecessary spinning)"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 10.0
      description: "Weight for twirling cost"

  VelocityDeadbandCritic:
    enabled:
      type: bool
      default_value: false
      description: "Enable VelocityDeadbandCritic"
    cost_power:
      type: int
      default_value: 1
      description: "Power exponent applied to cost"
    cost_weight:
      type: double
      default_value: 35.0
      description: "Weight for velocity deadband cost"
    deadband_velocities:
      type: double_array
      default_value: [0.05, 0.05, 0.05]
      description: "Deadband thresholds [vx, vy, wz]; velocities below are penalized"

```

---

### 📄 finenav_planner/finenav_mppi_controller/src/noise_generator.cpp

```cpp
// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

//  auto getParam = param_handler->getParamGetter(name);
//  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
```

---

### 📄 finenav_simulator/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_simulator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_executable(point_cloud_converter_node
        src/point_cloud_converter.cpp
)

ament_auto_add_executable(simulator_helper_node #TODO: executable
        src/simulator_helper_node.cpp
        src/keyboard_teleop.cpp
        src/odom_path_publisher.cpp
        src/ground_truth_map_localizer.cpp
)

ament_auto_package(
  # USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
        launch
        resource
        config
        scripts
)
```

---

### 📄 finenav_simulator/config/fastlio_gazebo_config.yaml

```yaml
/**:
    ros__parameters:
        feature_extract_enable: false
        point_filter_num: 1  # Keep dense sampling with the 32-line FAST-LIO input
        max_iteration: 3
        filter_size_surf: 0.05  # Slightly larger voxel size keeps simulation compute manageable
        filter_size_map: 0.05   # Match the short-range simulated LiDAR map density
        cube_side_length: 1000.0
        runtime_pos_log_enable: false
        map_file_path: "/tmp/fastlio_map.pcd"

        common:
            lid_topic:  "/cloud_for_lio"
            imu_topic:  "/imu"
            initial_frame: "odom"
            body_frame: "base_link"
            time_sync_en: false
            time_offset_lidar_to_imu: 0.0

            use_imu_odometry: true  # true = publish IMU odometry, false = publish LiDAR odometry
            imu_window_size_: 5

        preprocess:
            lidar_type: 2  # Velodyne-style PointCloud2 with ring/time fields
            scan_line: 64  # Use all 64 rings from the simulated LiDAR
            blind: 0.10
            timestamp_unit: 0  # point_cloud_converter synthesizes per-point time in seconds
            scan_rate: 10


        mapping:
            acc_cov: 0.1
            gyr_cov: 0.1
            b_acc_cov: 0.0001
            b_gyr_cov: 0.0001
            fov_degree: 360.0
            det_range: 7.0
            extrinsic_est_en: false
            extrinsic_T: [0.0, 0.0, 0.40]
            extrinsic_R: [1., 0., 0.,
                           0., 1., 0.,
                           0., 0., 1.]

        publish:
            path_en: true
            effect_map_en: true
            map_en: true
            scan_publish_en: true
            dense_publish_en: false
            scan_bodyframe_pub_en: true

        pcd_save:
            pcd_save_en: false
            interval: -1
```

---

### 📄 finenav_simulator/config/ros_gz_bridge.yaml

```yaml
# LiDAR point cloud
- ros_topic_name: "/livox/lidar"
  gz_topic_name: "/lidar/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "ignition.msgs.PointCloudPacked"
  direction: "GZ_TO_ROS"

# IMU
- ros_topic_name: "/imu"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "ignition.msgs.IMU"
  direction: "GZ_TO_ROS"

# Joint states
- ros_topic_name: "/joint_states"
  gz_topic_name: "/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"

# Velocity command
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "ignition.msgs.Twist"
  direction: "ROS_TO_GZ"

# Ground-truth odometry from Gazebo OdometryPublisher
- ros_topic_name: "/odom"
  gz_topic_name: "/ground_truth_odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: "GZ_TO_ROS"

# Keyboard input from Gazebo GUI
- ros_topic_name: "/keyboard/keypress"
  gz_topic_name: "/keyboard/keypress"
  ros_type_name: "std_msgs/msg/Int32"
  gz_type_name: "ignition.msgs.Int32"
  direction: "GZ_TO_ROS"
```

---

### 📄 finenav_simulator/config/world_spawn_poses.yaml

```yaml
world_spawn_poses:
  flat_small_scale_maze.world:
    x: 2.0
    y: 2.0
    z: 0.8
    yaw: 0.0

  map_desert.world:
    x: 0.7044101357460022
    y: 2.402442216873169
    z: 1.1844378113746643
    yaw: 0.0

  map_hill.world:
    x: 0.7596538066864014
    y: -2.1150081157684326
    z: 1.986343502998352
    yaw: 0.0

  map_vocano.world:
    x: 0.5622250437736511
    y: -1.1723533868789673
    z: 3.053374767303467
    yaw: 0.0
```

---

### 📄 finenav_simulator/include/finenav_simulator/ground_truth_map_localizer.hpp

```cpp
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace finenav_2d
{

class GroundTruthMapPublisher
{
public:
  explicit GroundTruthMapPublisher(rclcpp::Node * node);

private:
  void PublishIdentityMapToOdom() const;

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  bool enable_identity_map_to_odom_{false};
  std::string map_frame_;
  std::string odom_frame_;
};

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/include/finenav_simulator/keyboard_teleop.hpp

```cpp
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace finenav_2d
{

class KeyboardTeleop
{
public:
  explicit KeyboardTeleop(rclcpp::Node * node);

private:
  void OnKeypress(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node * node_;
  std::string teleop_input_topic_;
  std::string teleop_output_topic_;
  double linear_speed_;
  double angular_speed_;
  double linear_step_;
  double angular_step_;
  double min_linear_speed_;
  double max_linear_speed_;
  double min_angular_speed_;
  double max_angular_speed_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr key_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
};

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/include/finenav_simulator/odom_path_publisher.hpp

```cpp
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace finenav_2d
{

class OdomPathPublisher
{
public:
  OdomPathPublisher(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster);

private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void TrimPath();

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string input_odom_topic_;
  std::string path_topic_;
  std::string path_odom_frame_;
  std::string path_base_frame_;
  int max_path_points_;
  bool publish_tf_;
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/include/finenav_simulator/point_cloud_converter.hpp

```cpp
// Copyright 2025 Lihan Chen
//
// Licensed under the Mozilla Public License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.mozilla.org/en-US/MPL/2.0/
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                 uint16_t, ring, ring)(float, time, time))

namespace ign_sim_pointcloud_tool
{
class PointCloudConverter : public rclcpp::Node
{
public:
  explicit PointCloudConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);
  bool LookupOutputTransform(
    const std::string & source_frame, const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform);
  void ApplyFrameTransform(
    pcl::PointCloud<PointXYZIRT>::Ptr & cloud,
    const geometry_msgs::msg::TransformStamped & transform);

  template <typename T>
  void publishPoints(
    const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  int vertical_samples_;
  int horizontal_samples_;
  float vertical_min_angle_;
  float vertical_max_angle_;
  float horizontal_min_angle_;
  float horizontal_max_angle_;
  float scan_period_;
  bool synthesize_time_;
  bool self_filter_box_enable_;
  float self_filter_box_min_x_;
  float self_filter_box_max_x_;
  float self_filter_box_min_y_;
  float self_filter_box_max_y_;
  float self_filter_box_min_z_;
  float self_filter_box_max_z_;
  std::string pcd_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool transform_output_to_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace ign_sim_pointcloud_tool
```

---

### 📄 finenav_simulator/include/finenav_simulator/simulator_helper_node.hpp

```cpp
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "finenav_simulator/ground_truth_map_localizer.hpp"
#include "finenav_simulator/keyboard_teleop.hpp"
#include "finenav_simulator/odom_path_publisher.hpp"

namespace finenav_2d
{

class SimulatorHelper
{
public:
  explicit SimulatorHelper(std::shared_ptr<rclcpp::Node> node);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<KeyboardTeleop> teleop_;
  std::unique_ptr<OdomPathPublisher> path_publisher_;
  std::unique_ptr<GroundTruthMapPublisher> publisher_;
};

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/launch/complete/finenav_simulator.launch.py

```python
#  Copyright (c) 2025.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    sim_dir = get_package_share_directory('finenav_simulator')
    launch_dir = os.path.join(sim_dir, 'launch', 'complete')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose localization chain: fast_lio or gazebo_odom (legacy aliases ground_truth/static now fall back to gazebo_odom)'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='flat_small_scale_maze.world',
        description='World file name under finenav_simulator/resource/worlds'
    )

    declare_publish_map_tf = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='false',
        description='Enable publishing map to odom TF transform'
    )

    # Gazebo simulation and point-cloud pipeline
    simulator_infrastructure = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulator_infrastructure.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lio_type': LaunchConfiguration('lio_type'),
            'world_name': LaunchConfiguration('world_name'),
        }.items()
    )

    # FAST-LIO and Gazebo odometry support
    slam_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam_and_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lio_type': LaunchConfiguration('lio_type'),
            'publish_map_tf': LaunchConfiguration('publish_map_tf'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_world_name)
    ld.add_action(declare_publish_map_tf)
    ld.add_action(simulator_infrastructure)
    ld.add_action(slam_and_localization)

    return ld
```

---

### 📄 finenav_simulator/launch/complete/simulator_infrastructure.launch.py

```python
# Copyright (c) 2024.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_sim = get_package_share_directory('finenav_simulator')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose localization chain: fast_lio or gazebo_odom (legacy aliases ground_truth/static now fall back to gazebo_odom)'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='flat_small_scale_maze.world',
        description='World file name under finenav_simulator/resource/worlds'
    )

    # Gazebo environment
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_sim, 'launch', 'complete', 'spawn_world_and_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lio_type': LaunchConfiguration('lio_type'),
            'world_name': LaunchConfiguration('world_name'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_world_name)
    ld.add_action(gazebo_launch)

    return ld
```

---

### 📄 finenav_simulator/launch/complete/slam_and_localization.launch.py

```python
# Copyright (c) 2024.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('finenav_simulator'), 'config')

    fast_lio_condition = LaunchConfigurationEquals('lio_type', 'fast_lio')
    
    # static 模式下不需要 map 到 odom 的静态 TF
    dynamic_mode_condition = LaunchConfigurationEquals('navigation_mode', 'dynamic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose localization chain: fast_lio or gazebo_odom (legacy aliases ground_truth/static now fall back to gazebo_odom)'
    )

    declare_nav_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='dynamic',
        description='Choose the mode of navigation: dynamic or static'
    )

    declare_map_save = DeclareLaunchArgument(
        'map_save',
        default_value='false',
        description='Enable octomap saving after navigation'
    )

    declare_map_load = DeclareLaunchArgument(
        'map_load',
        default_value='false',
        description='Enable octomap loading a map before navigation'
    )

    declare_enable_octomap = DeclareLaunchArgument(
        'enable_octomap',
        default_value='true',
        description='Enable octomap server when running the fast_lio pipeline'
    )

    declare_publish_map_tf = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='false',
        description='Enable publishing map to odom TF transform'
    )

    fast_lio_config = os.path.join(config_dir, 'fastlio_gazebo_config.yaml')
    fast_lio_params = [
        fast_lio_config,
        {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }
    ]

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=fast_lio_params,
        output='screen',
        condition=fast_lio_condition
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='both',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': 0.05,
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'latch': False,
            'occupancy_min_z': 0.15,
            'occupancy_max_z': 7.45,
            'sensor_model.max_range': 7.0,
            'point_cloud_max_z': 7.45,
            'filter_ground_plane': False,
            'sensor_height': 0.40,
            'r_min': 0.2,
            'r_max': 7.0,
            'n_bins': 120,
            'n_segments': 360,
            'max_dist_to_line': 0.05,
            'min_slope': 0.0,
            'max_slope': 0.3,
            'max_fit_error': 0.05,
            'long_threshold': 1.0,
            'max_long_height': 0.1,
            'max_start_height': 0.2,
            'line_search_angle': 0.1,
            'visualize': False,
        }],
        remappings=[('/cloud_in', '/cloud_registered')],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('lio_type'), "' == 'fast_lio' and '",
                LaunchConfiguration('enable_octomap'), "' == 'true'"
            ])
        ),
    )

    map_to_odom_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('lio_type'), "' == 'fast_lio' and '",
                LaunchConfiguration('navigation_mode'), "' == 'dynamic' and '",
                LaunchConfiguration('publish_map_tf'), "' == 'true'"
            ])
        ),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_map_save)
    ld.add_action(declare_map_load)
    ld.add_action(declare_enable_octomap)
    ld.add_action(declare_publish_map_tf)
    ld.add_action(declare_nav_mode)
    ld.add_action(map_to_odom_static_node)
    ld.add_action(fast_lio_node)
    ld.add_action(octomap_server_node)

    return ld
```

---

### 📄 finenav_simulator/launch/complete/spawn_world_and_robot.launch.py

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('finenav_simulator')

    declare_lio_type = DeclareLaunchArgument(
        'lio_type',
        default_value='fast_lio',
        description='Choose localization chain: fast_lio or gazebo_odom (legacy aliases ground_truth/static now fall back to gazebo_odom)'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='flat_small_scale_maze.world',
        description='World file name under finenav_simulator/resource/worlds'
    )

    world_sdf_path = PathJoinSubstitution([
        pkg,
        'resource',
        'worlds',
        LaunchConfiguration('world_name'),
    ])
    ign_config_path = os.path.join(pkg, 'resource', 'ign', 'gui.config')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "core", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    )

    include_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "core", "robot.launch.py")
        ),
        launch_arguments={
            'lio_type': LaunchConfiguration('lio_type'),
            'world_name': LaunchConfiguration('world_name'),
        }.items(),
    )

    pointcloud_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "core", "pointcloud_complete_launch.py")
                ),
                launch_arguments={
                    'lio_type': LaunchConfiguration('lio_type'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        declare_lio_type,
        declare_world_name,
        gazebo_launch,
        include_robot_launch,
        pointcloud_launch
    ])
```

---

### 📄 finenav_simulator/launch/core/gazebo.launch.py

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("finenav_simulator")

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        description="Path to the world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    append_enviroment_worlds = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(pkg, "resource", "worlds"),
    )

    append_enviroment_models = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.join(pkg, "resource", "model"),
    )

    append_enviroment_models_gz = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg, "resource", "model"),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": [
                world_sdf_path,
                TextSubstitution(text=" --gui-config "),
                ign_config_path,
            ],
        }.items(),
    )

    # Bridge for simulation clock. 
    # We use multiple possible mappings to ensure compatibility with different ros_gz_bridge versions.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(append_enviroment_worlds)
    ld.add_action(append_enviroment_models)
    ld.add_action(append_enviroment_models_gz)
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)

    return ld
```

---

### 📄 finenav_simulator/launch/core/pointcloud_complete_launch.py

```python
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory("finenav_simulator")
scripts_dir = os.path.join(pkg_share_dir, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from sdf_launch_utils import (
    build_tf_args,
    generate_robot_sdf_from_xmacro,
    load_robot_sdf_config_from_string,
)


def generate_launch_description():
    pkg = get_package_share_directory("finenav_simulator")
    sdf_xmacro_path = os.path.join(pkg, "resource", "robot", "model.sdf.xmacro")
    lidar_support_mesh_uri = f"file://{os.path.join(pkg, 'resource', 'robot', 'meshes', 'lidar_support_tri_prism.obj')}"
    robot_xml = generate_robot_sdf_from_xmacro(
        sdf_xmacro_path,
        mappings={"lidar_support_mesh_uri": lidar_support_mesh_uri},
    )
    sdf_config = load_robot_sdf_config_from_string(robot_xml)
    robot_name = "fine_robot"

    declare_lio_type = DeclareLaunchArgument(
        "lio_type",
        default_value="fast_lio",
        description="Choose localization chain: fast_lio or gazebo_odom",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    gpu_lidar_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gpu_lidar_frame_alias',
        arguments=build_tf_args(
            'lidar_frame',
            f"{robot_name}/{sdf_config['lidar_parent_link']}/{sdf_config['lidar_sensor_name']}",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    converter_node = Node(
        package='finenav_simulator',
        executable='point_cloud_converter_node',
        name='point_cloud_converter',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'pcd_topic': '/livox/lidar'},
            {'output_topic': '/cloud_for_lio'},
            {
                'output_frame_id': PythonExpression([
                    "'lidar_frame' if '", LaunchConfiguration('lio_type'),
                    "' == 'fast_lio' else 'base_link'"
                ])
            },
            {
                'transform_output_to_frame': PythonExpression([
                    "'False' if '", LaunchConfiguration('lio_type'),
                    "' == 'fast_lio' else 'True'"
                ])
            },
            {'vertical_samples': sdf_config['lidar_vertical_samples']},
            {'horizontal_samples': sdf_config['lidar_horizontal_samples']},
            {'vertical_min_angle': sdf_config['lidar_vertical_min_angle']},
            {'vertical_max_angle': sdf_config['lidar_vertical_max_angle']},
            {'horizontal_min_angle': sdf_config['lidar_horizontal_min_angle']},
            {'horizontal_max_angle': sdf_config['lidar_horizontal_max_angle']},
            {'scan_period': sdf_config['lidar_scan_period']},
            {'synthesize_time': True},
        ]
    )

    return LaunchDescription([
        declare_lio_type,
        declare_use_sim_time,
        gpu_lidar_frame_alias,
        converter_node,
    ])
```

---

### 📄 finenav_simulator/launch/core/robot.launch.py

```python
import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory("finenav_simulator")
scripts_dir = os.path.join(pkg_share_dir, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from sdf_launch_utils import (
    build_tf_args,
    generate_robot_sdf_from_xmacro,
    generate_robot_urdf_from_sdf,
    load_robot_sdf_config_from_string,
    normalize_robot_urdf_for_rviz,
)


def _load_spawn_pose(config_path, world_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}

    poses = config.get("world_spawn_poses", config)
    pose = poses.get(world_name)
    if pose is None:
        pose = poses.get(os.path.splitext(world_name)[0])

    if pose is None:
        available = ", ".join(sorted(poses.keys()))
        raise RuntimeError(
            f"No spawn pose configured for world '{world_name}'. Available entries: {available}"
        )

    return {
        "x": str(float(pose["x"])),
        "y": str(float(pose["y"])),
        "z": str(float(pose["z"])),
        "yaw": str(float(pose.get("yaw", 0.0))),
    }


def _launch_setup(context):
    pkg = get_package_share_directory("finenav_simulator")

    sdf_xmacro_path = os.path.join(pkg, "resource", "robot", "model.sdf.xmacro")
    lidar_support_mesh_uri = f"file://{os.path.join(pkg, 'resource', 'robot', 'meshes', 'lidar_support_tri_prism.obj')}"
    bridge_config = os.path.join(pkg, "config", "ros_gz_bridge.yaml")
    spawn_pose_config = os.path.join(pkg, "config", "world_spawn_poses.yaml")

    robot_name = "fine_robot"
    world_name = LaunchConfiguration("world_name").perform(context)
    spawn_pose = _load_spawn_pose(spawn_pose_config, world_name)
    x_pose = spawn_pose["x"]
    y_pose = spawn_pose["y"]
    z_pose = spawn_pose["z"]
    yaw = spawn_pose["yaw"]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_xml = generate_robot_sdf_from_xmacro(
        sdf_xmacro_path,
        mappings={"lidar_support_mesh_uri": lidar_support_mesh_uri},
    )
    robot_urdf_xml = normalize_robot_urdf_for_rviz(generate_robot_urdf_from_sdf(robot_xml))
    sdf_config = load_robot_sdf_config_from_string(robot_xml)

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_xml,
            "-name", robot_name,
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-Y", yaw,
        ],
    )

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_name,
        parameters=[{"config_file": bridge_config}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        remappings=remappings,
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_urdf_xml,
        }],
    )

    imu_sensor_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_sensor_frame_alias",
        arguments=build_tf_args(
            "imu",
            f"{robot_name}/{sdf_config['imu_parent_link']}/{sdf_config['imu_sensor_name']}",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        parameters=[{"use_sim_time": True}],
    )

    simulator_helper = Node(
        package="finenav_simulator",
        executable="simulator_helper_node",
        parameters=[{
            "use_sim_time": True,
            "input_odom_topic": PythonExpression(["'/Odometry' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else '/odom'"]),
            "path_topic": "/path",
            "max_path_points": 2000,
            "publish_tf": PythonExpression(["'False' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else 'True'"]),
            "path_odom_frame": "odom",
            "path_base_frame": "base_link",
            "teleop_input_topic": "/keyboard/keypress",
            "teleop_output_topic": "/cmd_vel",
            "initial_linear_speed": 0.5,
            "initial_angular_speed": 1.0,
            "linear_step": 0.1,
            "angular_step": 0.1,
            "min_linear_speed": 0.0,
            "max_linear_speed": 2.0,
            "min_angular_speed": 0.0,
            "max_angular_speed": 3.0,
            "enable_keyboard_teleop": True,
            "enable_path_publisher": True,
            "enable_identity_map_to_odom": PythonExpression([
                "'False' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else 'True'"
            ]),
        }],
    )

    return [
        spawn_robot,
        robot_ign_bridge,
        robot_state_publisher,
        imu_sensor_alias,
        simulator_helper,
    ]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "lio_type",
            default_value="fast_lio",
            description="Choose localization chain: fast_lio or gazebo_odom",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_name",
            default_value="flat_small_scale_maze.world",
            description="World file name under finenav_simulator/resource/worlds",
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
```

---

### 📄 finenav_simulator/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_simulator</name>
    <version>1.0.0</version>
    <description>FineNav independent simulation package. Provides Gazebo environment, robot model, pointcloud tools and odometry for simulation.</description>
    <maintainer email="021424216@sues.edu.cn">Chenghao Wang</maintainer>
    <license>MPL-2.0</license>
    <buildtool_depend>ament_cmake_auto</buildtool_depend>
    <exec_depend>ament_index_python</exec_depend>
    <exec_depend>launch_ros</exec_depend>
    <exec_depend>robot_state_publisher</exec_depend>
    <depend>ros_gz_sim</depend>
    <depend>ros_gz_bridge</depend>
    <depend>pcl_conversions</depend>
    <depend>tf2_geometry_msgs</depend>
    <depend>nav_msgs</depend>
    <depend>geometry_msgs</depend>
    <depend>tf2_ros</depend>
    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_simulator/scripts/sdf_launch_utils.py

```python
#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import math
import copy
import xml.etree.ElementTree as ET

import xacro
from thirdparty.sdformat_tools.urdf_generator import UrdfGenerator
from thirdparty.xmacro.xmacro4sdf import XMLMacro4sdf


def _identity_pose():
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _parse_pose(pose_elem):
    if pose_elem is None or pose_elem.text is None:
        return _identity_pose(), None

    values = [float(v) for v in pose_elem.text.split()]
    if len(values) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(values)} from '{pose_elem.text}'")
    return tuple(values), pose_elem.get("relative_to")


def _parse_int_text(text, default="0"):
    value = text if text is not None else default
    return int(float(value))


def _rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_multiply(a, b):
    return [
        [
            a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def _matrix_vector_multiply(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def _matrix_transpose(matrix):
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def _matrix_to_rpy(matrix):
    if abs(matrix[2][0]) < 1.0:
        pitch = math.asin(-matrix[2][0])
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
        return roll, pitch, yaw

    pitch = math.pi / 2 if matrix[2][0] <= -1.0 else -math.pi / 2
    roll = math.atan2(-matrix[0][1], matrix[1][1])
    yaw = 0.0
    return roll, pitch, yaw


def _compose_pose(parent_pose, child_pose):
    parent_rotation = _rpy_to_matrix(*parent_pose[3:])
    child_rotation = _rpy_to_matrix(*child_pose[3:])
    child_translation = list(child_pose[:3])

    rotated_child = _matrix_vector_multiply(parent_rotation, child_translation)
    translation = tuple(parent_pose[i] + rotated_child[i] for i in range(3))
    rotation = _matrix_to_rpy(_matrix_multiply(parent_rotation, child_rotation))
    return (*translation, *rotation)


def _invert_pose(pose):
    rotation = _rpy_to_matrix(*pose[3:])
    rotation_inv = _matrix_transpose(rotation)
    translation = [-pose[0], -pose[1], -pose[2]]
    translated = _matrix_vector_multiply(rotation_inv, translation)
    rotation_rpy = _matrix_to_rpy(rotation_inv)
    return (*translated, *rotation_rpy)


def _relative_pose(parent_pose, child_pose):
    return _compose_pose(_invert_pose(parent_pose), child_pose)


def _find_link_sensor(model, sensor_name):
    for link in model.findall("link"):
        sensor = link.find(f"./sensor[@name='{sensor_name}']")
        if sensor is not None:
            return link, sensor
    raise KeyError(f"Could not find sensor '{sensor_name}' in SDF model")


def _build_entity_graph(model):
    entities = {}
    fixed_joint_children = {}

    for joint in model.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        child = joint.findtext("child")
        parent = joint.findtext("parent")
        if not child or not parent:
            continue
        joint_pose, joint_relative_to = _parse_pose(joint.find("pose"))
        fixed_joint_children[child] = {
            "parent": joint_relative_to or parent,
            "pose": joint_pose,
        }

    for link in model.findall("link"):
        pose, relative_to = _parse_pose(link.find("pose"))
        parent = relative_to
        if parent is None and link.get("name") in fixed_joint_children:
            parent = fixed_joint_children[link.get("name")]["parent"]
            pose = fixed_joint_children[link.get("name")]["pose"]
        entities[link.get("name")] = {
            "parent": parent,
            "pose": pose,
        }

    for frame in model.findall("frame"):
        pose, relative_to = _parse_pose(frame.find("pose"))
        parent = relative_to or frame.get("attached_to")
        entities[frame.get("name")] = {
            "parent": parent,
            "pose": pose,
        }

    for link in model.findall("link"):
        link_name = link.get("name")
        for sensor in link.findall("sensor"):
            pose, relative_to = _parse_pose(sensor.find("pose"))
            entities[sensor.get("name")] = {
                "parent": relative_to or link_name,
                "pose": pose,
            }

    return entities


def _resolve_entity_pose(entities, entity_name, cache=None):
    if cache is None:
        cache = {}
    if entity_name in cache:
        return cache[entity_name]
    if entity_name not in entities:
        raise KeyError(f"Unknown SDF entity '{entity_name}'")

    entity = entities[entity_name]
    if entity["parent"] is None:
        cache[entity_name] = entity["pose"]
        return cache[entity_name]

    parent_pose = _resolve_entity_pose(entities, entity["parent"], cache)
    cache[entity_name] = _compose_pose(parent_pose, entity["pose"])
    return cache[entity_name]


def _pose_to_tf_args(parent_frame, child_frame, pose):
    x, y, z, roll, pitch, yaw = pose
    return [
        "--x", f"{x}",
        "--y", f"{y}",
        "--z", f"{z}",
        "--roll", f"{roll}",
        "--pitch", f"{pitch}",
        "--yaw", f"{yaw}",
        "--frame-id", parent_frame,
        "--child-frame-id", child_frame,
    ]


def load_robot_sdf_config(sdf_path):
    with open(sdf_path, "r") as sdf_file:
        return load_robot_sdf_config_from_string(sdf_file.read())


def build_tf_args(parent_frame, child_frame, pose):
    return _pose_to_tf_args(parent_frame, child_frame, pose)


def generate_robot_sdf(xacro_path, mappings=None):
    doc = xacro.process_file(xacro_path, mappings=mappings or {})
    return doc.toxml()


def generate_robot_sdf_from_xmacro(xmacro_path, mappings=None):
    xmacro_generator = XMLMacro4sdf()
    xmacro_generator.set_xml_file(xmacro_path)
    xmacro_generator.generate(mappings or {})
    return xmacro_generator.to_string()


def generate_robot_urdf_from_sdf(sdf_xml):
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(sdf_xml)
    return urdf_generator.to_string()


def _parse_urdf_vector(origin_elem, attribute_name):
    if origin_elem is None:
        return (0.0, 0.0, 0.0)
    values = origin_elem.get(attribute_name, "0 0 0").split()
    return tuple(float(v) for v in values)


def _set_urdf_origin(origin_elem, xyz=None, rpy=None):
    if xyz is not None:
        origin_elem.set("xyz", " ".join(f"{value:g}" for value in xyz))
    if rpy is not None:
        origin_elem.set("rpy", " ".join(f"{value:g}" for value in rpy))


def _ensure_origin(parent_elem):
    origin = parent_elem.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        parent_elem.insert(0, origin)
    return origin


def normalize_robot_urdf_for_rviz(urdf_xml):
    root = ET.fromstring(urdf_xml)

    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}

    base_origin_link = links.get("base_origin")
    base_link = links.get("base_link")
    base_origin_to_base_link = joints.get("base_origin_to_base_link")
    if base_origin_link is None or base_link is None or base_origin_to_base_link is None:
        return urdf_xml

    base_shift_xyz = _parse_urdf_vector(base_origin_to_base_link.find("origin"), "xyz")
    # Shift content from the Gazebo body link into base_link so RViz keeps a valid TF tree.
    base_shift = tuple(-value for value in base_shift_xyz)

    for child in list(base_link):
        base_link.remove(child)

    for child_tag in ("inertial", "collision", "visual"):
        child_elem = base_origin_link.find(child_tag)
        if child_elem is None:
            continue
        child_copy = copy.deepcopy(child_elem)
        origin = _ensure_origin(child_copy)
        xyz = _parse_urdf_vector(origin, "xyz")
        _set_urdf_origin(origin, xyz=(xyz[0] + base_shift[0], xyz[1] + base_shift[1], xyz[2] + base_shift[2]))
        base_link.append(child_copy)

    base_lidar_link = links.get("base_lidar")
    base_lidar_joint = joints.get("base_lidar_joint")
    if base_lidar_link is not None and base_lidar_joint is not None:
        link_origin = base_lidar_link.find("origin")
        if link_origin is not None:
            base_lidar_link.remove(link_origin)
        base_lidar_joint.find("parent").set("link", "base_link")
        joint_origin = _ensure_origin(base_lidar_joint)
        xyz = _parse_urdf_vector(joint_origin, "xyz")
        _set_urdf_origin(joint_origin, xyz=(xyz[0] + base_shift[0], xyz[1] + base_shift[1], xyz[2] + base_shift[2]))

    for joint_name, link_name in (
        ("left_front_joint", "left_front_link"),
        ("left_rear_joint", "left_rear_link"),
        ("right_front_joint", "right_front_link"),
        ("right_rear_joint", "right_rear_link"),
    ):
        joint = joints.get(joint_name)
        link = links.get(link_name)
        if joint is None or link is None:
            continue

        link_origin = link.find("origin")
        if link_origin is None:
            continue

        xyz = _parse_urdf_vector(link_origin, "xyz")
        rpy = _parse_urdf_vector(link_origin, "rpy")
        link.remove(link_origin)

        for child_tag in ("inertial", "visual", "collision"):
            child_elem = link.find(child_tag)
            if child_elem is None:
                continue
            child_origin = _ensure_origin(child_elem)
            _set_urdf_origin(child_origin, xyz=(0.0, 0.0, 0.0), rpy=rpy)

        joint.find("parent").set("link", "base_link")
        joint_origin = _ensure_origin(joint)
        _set_urdf_origin(
            joint_origin,
            xyz=(xyz[0] + base_shift[0], xyz[1] + base_shift[1], xyz[2] + base_shift[2]),
            rpy=(0.0, 0.0, 0.0),
        )

    root.remove(base_origin_link)
    root.remove(base_origin_to_base_link)

    return ET.tostring(root, encoding="unicode")


def load_robot_sdf_config_from_string(sdf_xml):
    root = ET.fromstring(sdf_xml)
    model = root.find("model")
    if model is None:
        raise RuntimeError("Could not find <model> in generated SDF XML")

    entities = _build_entity_graph(model)
    cache = {}

    base_link_pose = _resolve_entity_pose(entities, "base_link", cache)
    lidar_frame_pose = _resolve_entity_pose(entities, "lidar_frame", cache)
    lidar_parent_link, lidar_sensor = _find_link_sensor(model, "gpu_lidar")
    imu_parent_link, imu_sensor = _find_link_sensor(model, "imu_sensor")

    horizontal = lidar_sensor.find("./ray/scan/horizontal")
    vertical = lidar_sensor.find("./ray/scan/vertical")
    lidar_range = lidar_sensor.find("./ray/range")

    if horizontal is None or vertical is None or lidar_range is None:
        raise RuntimeError("Could not find complete GPU LiDAR scan configuration in SDF")

    update_rate = float(lidar_sensor.findtext("update_rate", default="0") or 0.0)
    scan_period = 1.0 / update_rate if update_rate > 0.0 else 0.1

    return {
        "base_link_to_lidar": _relative_pose(base_link_pose, lidar_frame_pose),
        "lidar_parent_link": lidar_parent_link.get("name"),
        "lidar_sensor_name": lidar_sensor.get("name"),
        "imu_parent_link": imu_parent_link.get("name"),
        "imu_sensor_name": imu_sensor.get("name"),
        "lidar_vertical_samples": _parse_int_text(vertical.findtext("samples", default="0")),
        "lidar_horizontal_samples": _parse_int_text(horizontal.findtext("samples", default="0")),
        "lidar_vertical_min_angle": float(vertical.findtext("min_angle", default="0.0")),
        "lidar_vertical_max_angle": float(vertical.findtext("max_angle", default="0.0")),
        "lidar_horizontal_min_angle": float(horizontal.findtext("min_angle", default="0.0")),
        "lidar_horizontal_max_angle": float(horizontal.findtext("max_angle", default="0.0")),
        "lidar_min_range": float(lidar_range.findtext("min", default="0.0")),
        "lidar_max_range": float(lidar_range.findtext("max", default="0.0")),
        "lidar_scan_period": scan_period,
    }
```

---

### 📄 finenav_simulator/src/ground_truth_map_localizer.cpp

```cpp
#include "finenav_simulator/ground_truth_map_localizer.hpp"

#include <string>

namespace finenav_2d
{

namespace
{

template<typename T>
T DeclareOrGetParameter(rclcpp::Node * node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

}  // namespace

GroundTruthMapPublisher::GroundTruthMapPublisher(rclcpp::Node * node)
: node_(node),
  static_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(node))
{
  enable_identity_map_to_odom_ =
    DeclareOrGetParameter<bool>(node_, "enable_identity_map_to_odom", false);
  map_frame_ = DeclareOrGetParameter<std::string>(node_, "map_frame", "map");
  odom_frame_ = DeclareOrGetParameter<std::string>(node_, "odom_frame", "odom");

  if (enable_identity_map_to_odom_) {
    PublishIdentityMapToOdom();
  }
}

void GroundTruthMapPublisher::PublishIdentityMapToOdom() const
{
  geometry_msgs::msg::TransformStamped transform;
  // Use zero time for static transform to avoid synchronization issues
  // or jump-back-in-time errors when sim_time starts.
  transform.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(transform);
}

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/src/keyboard_teleop.cpp

```cpp
#include "finenav_simulator/keyboard_teleop.hpp"

#include <algorithm>
#include <string>
#include <cmath>

namespace finenav_2d
{

KeyboardTeleop::KeyboardTeleop(rclcpp::Node * node)
: node_(node)
{
  teleop_input_topic_ = node_->declare_parameter("teleop_input_topic", "/keyboard/keypress");
  teleop_output_topic_ = node_->declare_parameter("teleop_output_topic", "/cmd_vel");
  linear_speed_ = node_->declare_parameter("initial_linear_speed", 0.5);
  angular_speed_ = node_->declare_parameter("initial_angular_speed", 1.0);
  linear_step_ = node_->declare_parameter("linear_step", 0.1);
  angular_step_ = node_->declare_parameter("angular_step", 0.1);
  min_linear_speed_ = node_->declare_parameter("min_linear_speed", 0.0);
  max_linear_speed_ = node_->declare_parameter("max_linear_speed", 2.0);
  min_angular_speed_ = node_->declare_parameter("min_angular_speed", 0.0);
  max_angular_speed_ = node_->declare_parameter("max_angular_speed", 3.0);

  cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(teleop_output_topic_, 10);
  key_subscription_ = node_->create_subscription<std_msgs::msg::Int32>(
    teleop_input_topic_, 10,
    std::bind(&KeyboardTeleop::OnKeypress, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "Keyboard teleop listening on %s and publishing %s",
    teleop_input_topic_.c_str(), teleop_output_topic_.c_str());
}

void KeyboardTeleop::OnKeypress(const std_msgs::msg::Int32::SharedPtr msg)
{
  bool should_publish = true;

  switch (msg->data) {
    case 87:  // W
      current_cmd_.linear.x = linear_speed_;
      current_cmd_.angular.z = 0.0;
      break;
    case 83:  // S
      current_cmd_.linear.x = -linear_speed_;
      current_cmd_.angular.z = 0.0;
      break;
    case 65:  // A
      current_cmd_.linear.x = 0.0;
      current_cmd_.angular.z = angular_speed_;
      break;
    case 68:  // D
      current_cmd_.linear.x = 0.0;
      current_cmd_.angular.z = -angular_speed_;
      break;
    case 81:  // Q
      angular_speed_ = std::min(max_angular_speed_, angular_speed_ + angular_step_);
      if (current_cmd_.angular.z > 0.0) {
        current_cmd_.angular.z = angular_speed_;
      } else if (current_cmd_.angular.z < 0.0) {
        current_cmd_.angular.z = -angular_speed_;
      }
      break;
    case 69:  // E
      angular_speed_ = std::max(min_angular_speed_, angular_speed_ - angular_step_);
      if (current_cmd_.angular.z > 0.0) {
        current_cmd_.angular.z = angular_speed_;
      } else if (current_cmd_.angular.z < 0.0) {
        current_cmd_.angular.z = -angular_speed_;
      }
      break;
    case 90:  // Z
      linear_speed_ = std::min(max_linear_speed_, linear_speed_ + linear_step_);
      if (current_cmd_.linear.x > 0.0) {
        current_cmd_.linear.x = linear_speed_;
      } else if (current_cmd_.linear.x < 0.0) {
        current_cmd_.linear.x = -linear_speed_;
      }
      break;
    case 67:  // C
      linear_speed_ = std::max(min_linear_speed_, linear_speed_ - linear_step_);
      if (current_cmd_.linear.x > 0.0) {
        current_cmd_.linear.x = linear_speed_;
      } else if (current_cmd_.linear.x < 0.0) {
        current_cmd_.linear.x = -linear_speed_;
      }
      break;
    case 88:  // X
      current_cmd_ = geometry_msgs::msg::Twist();
      break;
    default:
      should_publish = false;
      break;
  }

  if (!should_publish) {
    return;
  }

  cmd_publisher_->publish(current_cmd_);
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "linear_speed=%.2f angular_speed=%.2f cmd=(%.2f, %.2f)",
    linear_speed_, angular_speed_, current_cmd_.linear.x, current_cmd_.angular.z);
}

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/src/odom_path_publisher.cpp

```cpp
#include "finenav_simulator/odom_path_publisher.hpp"

#include <string>

namespace finenav_2d
{

OdomPathPublisher::OdomPathPublisher(rclcpp::Node * node, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster)
: node_(node), tf_broadcaster_(tf_broadcaster)
{
  input_odom_topic_ = node_->declare_parameter("input_odom_topic", "/odom");
  path_topic_ = node_->declare_parameter("path_topic", "/path");
  max_path_points_ = node_->declare_parameter("max_path_points", 2000);
  publish_tf_ = node_->declare_parameter("publish_tf", true);
  path_odom_frame_ = node_->declare_parameter("path_odom_frame", "");
  path_base_frame_ = node_->declare_parameter("path_base_frame", "");

  odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    std::bind(&OdomPathPublisher::OdomCallback, this, std::placeholders::_1));

  path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
}

void OdomPathPublisher::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const std::string odom_frame = path_odom_frame_.empty() ? msg->header.frame_id : path_odom_frame_;
  const std::string base_frame = path_base_frame_.empty() ? msg->child_frame_id : path_base_frame_;

  if (odom_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Received odometry without a frame_id, skipping path update.");
    return;
  }

  if (publish_tf_ && base_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Received odometry without a child_frame_id, skipping TF publication.");
  }

  if (publish_tf_ && !base_frame.empty()) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = msg->header;
    odom_tf.header.frame_id = odom_frame;
    odom_tf.child_frame_id = base_frame;
    odom_tf.transform.translation.x = msg->pose.pose.position.x;
    odom_tf.transform.translation.y = msg->pose.pose.position.y;
    odom_tf.transform.translation.z = msg->pose.pose.position.z;
    odom_tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
  }

  if (!path_.header.frame_id.empty() && path_.header.frame_id != odom_frame) {
    path_.poses.clear();
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = msg->header;
  pose_stamped.header.frame_id = odom_frame;
  pose_stamped.pose = msg->pose.pose;

  path_.header = msg->header;
  path_.header.frame_id = odom_frame;
  path_.poses.push_back(pose_stamped);
  this->TrimPath();

  path_publisher_->publish(path_);
}

void OdomPathPublisher::TrimPath()
{
  if (max_path_points_ <= 0) {
    return;
  }

  if (path_.poses.size() <= static_cast<size_t>(max_path_points_)) {
    return;
  }

  const auto erase_count = path_.poses.size() - static_cast<size_t>(max_path_points_);
  path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<std::ptrdiff_t>(erase_count));
}

}  // namespace finenav_2d
```

---

### 📄 finenav_simulator/src/point_cloud_converter.cpp

```cpp
// Copyright 2025 Lihan Chen
//
// Licensed under the Mozilla Public License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.mozilla.org/en-US/MPL/2.0/
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_simulator/point_cloud_converter.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace
{
constexpr float kTwoPi = 2.0F * static_cast<float>(M_PI);

bool HasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name)
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name;
    });
}

size_t ClampIndex(int index, int size)
{
  if (size <= 1) {
    return 0;
  }
  return static_cast<size_t>(std::clamp(index, 0, size - 1));
}

float NormalizeAnglePositive(float angle)
{
  float normalized = std::fmod(angle, kTwoPi);
  if (normalized < 0.0F) {
    normalized += kTwoPi;
  }
  return normalized;
}

size_t ComputeRingFromAngle(
  float x, float y, float z, float vertical_min_angle, float vertical_max_angle,
  int vertical_samples)
{
  if (vertical_samples <= 1) {
    return 0;
  }

  const float vertical_span = vertical_max_angle - vertical_min_angle;
  if (vertical_span <= 0.0F) {
    return 0;
  }

  const float vertical_step = vertical_span / static_cast<float>(vertical_samples - 1);
  const float vertical_angle = std::atan2(z, std::hypot(x, y));
  const int ring = static_cast<int>(std::lround((vertical_angle - vertical_min_angle) / vertical_step));
  return ClampIndex(ring, vertical_samples);
}

size_t ComputeColumnFromAngle(
  float x, float y, float horizontal_min_angle, float horizontal_max_angle,
  int horizontal_samples)
{
  if (horizontal_samples <= 1) {
    return 0;
  }

  const float horizontal_span = horizontal_max_angle - horizontal_min_angle;
  if (horizontal_span <= 0.0F) {
    return 0;
  }

  const float horizontal_step = horizontal_span / static_cast<float>(horizontal_samples);
  const float azimuth = NormalizeAnglePositive(std::atan2(y, x) - horizontal_min_angle);
  const int column = static_cast<int>(std::floor(azimuth / horizontal_step));
  return ClampIndex(column, horizontal_samples);
}

float ComputeRelativeTime(size_t column, int horizontal_samples, float scan_period)
{
  if (horizontal_samples <= 0) {
    return 0.0F;
  }
  return static_cast<float>(column) * scan_period / static_cast<float>(horizontal_samples);
}

bool IsInsideBox(
  float x, float y, float z,
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z)
{
  return x >= min_x && x <= max_x &&
         y >= min_y && y <= max_y &&
         z >= min_z && z <= max_z;
}

void ApplyTransformToPoint(
  const geometry_msgs::msg::TransformStamped & transform,
  float & x, float & y, float & z)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;

  tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
  tf2::Matrix3x3 basis(quaternion);
  const tf2::Vector3 point(x, y, z);
  const tf2::Vector3 transformed = basis * point;

  x = static_cast<float>(transformed.x() + translation.x);
  y = static_cast<float>(transformed.y() + translation.y);
  z = static_cast<float>(transformed.z() + translation.z);
}
}  // namespace

namespace ign_sim_pointcloud_tool
{
PointCloudConverter::PointCloudConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("point_cloud_converter", options)
{
  this->declare_parameter<std::string>("pcd_topic", "livox/lidar");
  this->declare_parameter<int>("vertical_samples", 0);
  this->declare_parameter<int>("horizontal_samples", 0);
  this->declare_parameter<float>("vertical_min_angle", -0.26F);
  this->declare_parameter<float>("vertical_max_angle", 1.57F);
  this->declare_parameter<float>("horizontal_min_angle", 0.0F);
  this->declare_parameter<float>("horizontal_max_angle", 6.2831852F);
  this->declare_parameter<int>("n_scan", 0);
  this->declare_parameter<int>("horizon_scan", 0);
  this->declare_parameter<float>("scan_period", 0.1F);
  this->declare_parameter<bool>("synthesize_time", true);
  this->declare_parameter<std::string>("output_topic", "/cloud_for_lio");
  this->declare_parameter<std::string>("output_frame_id", "");
  this->declare_parameter<bool>("transform_output_to_frame", false);
  // Self-filter box in lidar_frame, sized from the current robot body + wheels.
  this->declare_parameter<bool>("self_filter_box.enable", true);
  this->declare_parameter<float>("self_filter_box.min_x", -0.50F);
  this->declare_parameter<float>("self_filter_box.max_x", 0.26F);
  this->declare_parameter<float>("self_filter_box.min_y", -0.32F);
  this->declare_parameter<float>("self_filter_box.max_y", 0.32F);
  this->declare_parameter<float>("self_filter_box.min_z", -0.42F);
  this->declare_parameter<float>("self_filter_box.max_z", 0.05F);

  pcd_topic_ = this->get_parameter("pcd_topic").as_string();
  vertical_samples_ = this->get_parameter("vertical_samples").as_int();
  horizontal_samples_ = this->get_parameter("horizontal_samples").as_int();
  vertical_min_angle_ = this->get_parameter("vertical_min_angle").as_double();
  vertical_max_angle_ = this->get_parameter("vertical_max_angle").as_double();
  horizontal_min_angle_ = this->get_parameter("horizontal_min_angle").as_double();
  horizontal_max_angle_ = this->get_parameter("horizontal_max_angle").as_double();
  scan_period_ = this->get_parameter("scan_period").as_double();
  synthesize_time_ = this->get_parameter("synthesize_time").as_bool();
  output_topic_ = this->get_parameter("output_topic").as_string();
  output_frame_id_ = this->get_parameter("output_frame_id").as_string();
  transform_output_to_frame_ = this->get_parameter("transform_output_to_frame").as_bool();
  self_filter_box_enable_ = this->get_parameter("self_filter_box.enable").as_bool();
  self_filter_box_min_x_ = this->get_parameter("self_filter_box.min_x").as_double();
  self_filter_box_max_x_ = this->get_parameter("self_filter_box.max_x").as_double();
  self_filter_box_min_y_ = this->get_parameter("self_filter_box.min_y").as_double();
  self_filter_box_max_y_ = this->get_parameter("self_filter_box.max_y").as_double();
  self_filter_box_min_z_ = this->get_parameter("self_filter_box.min_z").as_double();
  self_filter_box_max_z_ = this->get_parameter("self_filter_box.max_z").as_double();

  if (vertical_samples_ <= 0) {
    vertical_samples_ = this->get_parameter("n_scan").as_int();
  }
  if (horizontal_samples_ <= 0) {
    horizontal_samples_ = this->get_parameter("horizon_scan").as_int();
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pcd_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointCloudConverter::lidarHandle, this, std::placeholders::_1));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

  RCLCPP_INFO(
    this->get_logger(),
    "Listening to lidar topic: %s, expected layout: %d x %d, h_angle:[%.4f, %.4f], v_angle:[%.4f, %.4f], scan_period: %.4f, synthesize_time: %s, output_topic: %s, output_frame_id: %s, transform_output_to_frame: %s, self_filter_box: %s [x: %.2f..%.2f, y: %.2f..%.2f, z: %.2f..%.2f]",
    pcd_topic_.c_str(), vertical_samples_, horizontal_samples_,
    horizontal_min_angle_, horizontal_max_angle_, vertical_min_angle_, vertical_max_angle_,
    scan_period_, synthesize_time_ ? "true" : "false", output_topic_.c_str(),
    output_frame_id_.empty() ? "<input>" : output_frame_id_.c_str(),
    transform_output_to_frame_ ? "true" : "false",
    self_filter_box_enable_ ? "enabled" : "disabled",
    self_filter_box_min_x_, self_filter_box_max_x_,
    self_filter_box_min_y_, self_filter_box_max_y_,
    self_filter_box_min_z_, self_filter_box_max_z_);
}

bool PointCloudConverter::LookupOutputTransform(
  const std::string & source_frame, const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_buffer_->lookupTransform(output_frame_id_, source_frame, stamp);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to transform point cloud from '%s' to '%s': %s",
      source_frame.c_str(), output_frame_id_.c_str(), ex.what());
    return false;
  }
}

void PointCloudConverter::ApplyFrameTransform(
  pcl::PointCloud<PointXYZIRT>::Ptr & cloud,
  const geometry_msgs::msg::TransformStamped & transform)
{
  for (auto & point : cloud->points) {
    ApplyTransformToPoint(transform, point.x, point.y, point.z);
  }
}

void PointCloudConverter::lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg)
{
  pcl::PointCloud<PointXYZIRT>::Ptr converted_pc(new pcl::PointCloud<PointXYZIRT>());
  const size_t point_count = static_cast<size_t>(pc_msg->width) * static_cast<size_t>(pc_msg->height);
  converted_pc->points.reserve(point_count);

  const bool organized_cloud = pc_msg->height > 1 && pc_msg->width > 1;
  int vertical_samples = vertical_samples_;
  int horizontal_samples = horizontal_samples_;

  if (organized_cloud) {
    if (
      vertical_samples_ > 0 && horizontal_samples_ > 0 &&
      (vertical_samples_ != static_cast<int>(pc_msg->height) ||
      horizontal_samples_ != static_cast<int>(pc_msg->width)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Configured GPU LiDAR layout %d x %d, but incoming organized cloud is %u x %u. Using incoming layout.",
        vertical_samples_, horizontal_samples_, pc_msg->height, pc_msg->width);
    }
    vertical_samples = static_cast<int>(pc_msg->height);
    horizontal_samples = static_cast<int>(pc_msg->width);
  }

  if (vertical_samples <= 0 || horizontal_samples <= 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Invalid lidar layout: vertical_samples=%d, horizontal_samples=%d",
      vertical_samples, horizontal_samples);
    return;
  }

  const size_t required_data_size =
    static_cast<size_t>(pc_msg->width) * static_cast<size_t>(pc_msg->height) *
    static_cast<size_t>(pc_msg->point_step);
  if (pc_msg->data.size() < required_data_size) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Malformed PointCloud2 data: data.size()=%zu, required=%zu",
      pc_msg->data.size(), required_data_size);
    return;
  }

  const bool has_intensity = HasField(*pc_msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<float> x_iter(*pc_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iter(*pc_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iter(*pc_msg, "z");
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> intensity_iter;
  if (has_intensity) {
    intensity_iter = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*pc_msg, "intensity");
  }

  bool removed_points = false;

  for (size_t point_id = 0; point_id < point_count; ++point_id, ++x_iter, ++y_iter, ++z_iter) {
    PointXYZIRT new_point{};
    new_point.x = *x_iter;
    new_point.y = *y_iter;
    new_point.z = *z_iter;
    new_point.intensity = has_intensity ? **intensity_iter : 0.0F;

    if (has_intensity) {
      ++(*intensity_iter);
    }

    if (
      self_filter_box_enable_ &&
      IsInsideBox(
        new_point.x, new_point.y, new_point.z,
        self_filter_box_min_x_, self_filter_box_max_x_,
        self_filter_box_min_y_, self_filter_box_max_y_,
        self_filter_box_min_z_, self_filter_box_max_z_))
    {
      removed_points = true;
      continue;
    }

    size_t ring = 0;
    size_t column = 0;

    if (organized_cloud) {
      column = point_id % static_cast<size_t>(horizontal_samples);
      ring = point_id / static_cast<size_t>(horizontal_samples);
    } else {
      ring = ComputeRingFromAngle(
        new_point.x, new_point.y, new_point.z,
        vertical_min_angle_, vertical_max_angle_, vertical_samples);
      if (synthesize_time_) {
        column = ComputeColumnFromAngle(
          new_point.x, new_point.y,
          horizontal_min_angle_, horizontal_max_angle_, horizontal_samples);
      }
    }

    new_point.ring = static_cast<uint16_t>(ring);
    new_point.time = synthesize_time_ ?
      ComputeRelativeTime(column, horizontal_samples, scan_period_) : 0.0F;
    converted_pc->points.push_back(new_point);
  }

  const bool keep_organized_output = organized_cloud && !removed_points;
  converted_pc->width = keep_organized_output ? static_cast<uint32_t>(horizontal_samples) :
    static_cast<uint32_t>(converted_pc->points.size());
  converted_pc->height = keep_organized_output ? static_cast<uint32_t>(vertical_samples) : 1;

  geometry_msgs::msg::TransformStamped output_transform;
  if (transform_output_to_frame_) {
    if (output_frame_id_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "transform_output_to_frame is enabled but output_frame_id is empty.");
      return;
    }
    if (pc_msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Incoming point cloud has empty frame_id; cannot transform to '%s'.",
        output_frame_id_.c_str());
      return;
    }
    if (!LookupOutputTransform(pc_msg->header.frame_id, pc_msg->header.stamp, output_transform)) {
      return;
    }
    ApplyFrameTransform(converted_pc, output_transform);
  }

  publishPoints<PointXYZIRT>(converted_pc, *pc_msg);
}

template <typename T>
void PointCloudConverter::publishPoints(
  const typename pcl::PointCloud<T>::Ptr & new_pc, const sensor_msgs::msg::PointCloud2 & old_msg)
{
  new_pc->is_dense = false;

  sensor_msgs::msg::PointCloud2 pc_new_msg;
  pcl::toROSMsg(*new_pc, pc_new_msg);
  pc_new_msg.header = old_msg.header;
  if (!output_frame_id_.empty()) {
    pc_new_msg.header.frame_id = output_frame_id_;
  }

  pcd_pub_->publish(pc_new_msg);
}

}  // namespace ign_sim_pointcloud_tool

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<ign_sim_pointcloud_tool::PointCloudConverter>(options));
  rclcpp::shutdown();
  return 0;
}
```

---

### 📄 finenav_simulator/src/simulator_helper_node.cpp

```cpp
#include "finenav_simulator/simulator_helper_node.hpp"

namespace finenav_2d
{

SimulatorHelper::SimulatorHelper(std::shared_ptr<rclcpp::Node> node)
: node_(node),
  tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*node_))
{
  teleop_ = std::make_unique<KeyboardTeleop>(node_.get());
  path_publisher_ = std::make_unique<OdomPathPublisher>(node_.get(), tf_broadcaster_);
  publisher_ = std::make_unique<GroundTruthMapPublisher>(node_.get());
}

}  // namespace finenav_2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("simulator_helper_node");
  auto helper = std::make_unique<finenav_2d::SimulatorHelper>(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

---

### 📄 finenav_util/CMakeLists.txt

```
cmake_minimum_required(VERSION 3.22)
project(finenav_util)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

#add_library(${PROJECT_NAME} INTERFACE)
#
#target_include_directories(${PROJECT_NAME} INTERFACE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/node_thread.cpp
        src/cloud_publish_helper.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package() # TODO: 之后讨论下是否添加 USE_SCOPED_HEADER_INSTALL_DIR 的问题
```

---

### 📄 finenav_util/include/finenav_util/algo_configurator.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

// TODO: 改名 xxxx_marcos
namespace finenav {
// 不需要配置的默认模版
template <typename AlgoT>
struct AlgoConfigurator {
    static constexpr std::string_view name = "unknown_algo";

    // 返回一个智能指针，将生命周期转移给调用者
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr, AlgoT&) {
        // TODO: 这种情况应该警告，表示用户没有注册算法
        return nullptr; // 返回空指针，keep_alive 会自动忽略
    }
};
}

// TODO： 考虑 PARAM_NS 是否改名为 algo_name 或者 algo_id 之类的东西，只要能够对用户屏蔽参数概念
// TODO: 后续还需要考虑动态参数问题
// 另外，可能会有重名问题？
#define FINENAV_REGISTER_ALGO_CONFIG(ALGO_TYPE, PARAM_NS) \
namespace finenav { \
template <> \
struct AlgoConfigurator<ALGO_TYPE> { \
    static constexpr std::string_view name = #PARAM_NS; \
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, ALGO_TYPE& algo) { \
        /* 1. 创建监听器 */ \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        /* 2. 初始配置 (立即生效) */ \
        algo.configure(listener->get_params()); \
        \
        /* 3. 注册参数更新回调 */ \
        listener->setUserCallback([&algo](const auto& params) { \
            algo.configure(params); \
        }); \
        \
        /* 4. 返回指针以维持生命周期 */ \
        return listener; \
    } \
}; \
}\

#define FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG(TEMPLATE_ALGO, PARAM_NS) \
namespace finenav { \
template <typename... Args> \
struct AlgoConfigurator<TEMPLATE_ALGO<Args...>> { \
    static constexpr std::string_view name = #PARAM_NS; \
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, TEMPLATE_ALGO<Args...>& algo) { \
        /* 1. 创建监听器 */ \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        /* 2. 初始配置 (立即生效) */ \
        algo.configure(listener->get_params()); \
        \
        /* 3. 注册参数更新回调 */ \
        listener->setUserCallback([&algo](const auto& params) { \
        algo.configure(params); \
        }); \
        \
        /* 4. 返回指针以维持生命周期 */ \
        return listener; \
    } \
};\
}\

```

---

### 📄 finenav_util/include/finenav_util/cloud_publish_helper.hpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace finenav_utils {

class CloudPublishHelper {
public:
    CloudPublishHelper() = default;
    ~CloudPublishHelper() = default;

    void configure(const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>& pub,
               bool is_dense, const std::string& frame_id);

    void addPoint(const float& x, const float& y, const float& z,
                  const std::array<uint8_t, 3>& rgb = {255, 255, 255});

    void reserve(size_t n);

    bool hasSubscribers() const;

    // 需要传入rclcpp的时间
    void publish(rclcpp::Time time);

private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
};

} // namespace finenav_utils
```

---

### 📄 finenav_util/include/finenav_util/node_thread.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace finenav {

class NodeThread {
  public:

    explicit NodeThread(rclcpp::Node::SharedPtr node);

    explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

    NodeThread(const NodeThread&) = delete;
    NodeThread& operator=(const NodeThread&) = delete;
    NodeThread(NodeThread&&) = delete;
    NodeThread& operator=(NodeThread&&) = delete;

    ~NodeThread();

  protected:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<std::thread> thread_;
    rclcpp::Executor::SharedPtr executor_;
};

}  // namespace finenav
```

---

### 📄 finenav_util/include/finenav_util/simple_action_server.hpp

```cpp
// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// Copyright (c) 2026 FineNav Team
// Based on the architectural concepts of Nav2 SimpleActionServer
// Specialized for lightweight SDK integration.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <functional>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace finenav {

/**
 * @brief FineNav 专用简化版 Action Server
 * * 核心职责：
 * 1. 维护 "Current" (当前) 和 "Pending" (挂起) 两个目标槽位。
 * 2. 处理多线程环境下的目标抢占 (Preemption) 逻辑。
 * 3. 将复杂的 Action 回调简化为单一的 execute_callback 循环。
 */
template<typename ActionT>
class SimpleActionServer
{
  public:
    // 定义回调函数类型，用户只需提供一个无参数的执行函数
    using ExecuteCallback = std::function<void()>;
    using GoalHandle = typename rclcpp_action::ServerGoalHandle<ActionT>;
    // using GoalHandlePtr = typename GoalHandle::SharedPtr;

    /**
     * @brief 构造函数
     * @param node ROS 节点句柄 (直接使用 rclcpp::Node，去除 nav2_util 依赖)
     * @param action_name Action 名字
     * @param execute_callback 用户的主执行逻辑
     */
    SimpleActionServer(rclcpp::Node::SharedPtr node, const std::string & action_name, ExecuteCallback execute_callback)
        : node_(node), action_name_(action_name), execute_callback_(execute_callback)
    {
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
            std::bind(&SimpleActionServer::handle_cancel, this, _1),
            std::bind(&SimpleActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(node_->get_logger(), "FineNav ActionServer [%s] started.", action_name_.c_str());
    }

    ~SimpleActionServer()
    {
        // 逻辑：析构时必须确保工作线程退出，否则会抛出 std::terminate
        {
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);
            stop_execution_ = true;
            if (current_handle_) {
                current_handle_->abort(std::make_shared<typename ActionT::Result>());
            }
        }
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    /**
     * @brief 检查是否有新目标正在排队（抢占请求）
     * 逻辑：用户在 execute 循环中必须频繁检查此标志。
     * 如果返回 true，用户应尽快退出当前循环，以便 worker 线程调度新目标。
     */
    bool is_preempt_requested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return pending_handle_ != nullptr;
    }

    /**
     * @brief 检查是否收到取消请求
     */
    bool is_cancel_requested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        // 逻辑：无论是当前目标被取消，还是排队目标被取消，都视为取消信号
        if (current_handle_ && current_handle_->is_canceling()) return true;
        if (pending_handle_ && pending_handle_->is_canceling()) return true;
        return false;
    }

    /**
     * @brief 获取当前正在执行的目标
     */
    std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!current_handle_ || !current_handle_->is_active()) {
            return nullptr;
        }
        return current_handle_->get_goal();
    }

    /**
     * @brief 接受挂起的目标 (执行抢占切换)
     * 逻辑：
     * 1. 终止当前目标 (Canceled/Aborted)
     * 2. 将 Pending 提升为 Current
     * 3. 返回新目标的 Handle 供用户获取数据
     */
     const std::shared_ptr<const typename ActionT::Goal>  accept_pending_goal()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] No pending goal to accept!", action_name_.c_str());
            return nullptr;
        }

        // 如果当前还有旧目标在跑，先把它结束掉
        if (current_handle_ && current_handle_->is_active() && current_handle_ != pending_handle_) {
            RCLCPP_INFO(node_->get_logger(), "[%s] Preempting current goal.", action_name_.c_str());
            auto result = std::make_shared<typename ActionT::Result>();
            current_handle_->abort(result); // 或者 canceled，视业务逻辑而定，通常抢占视为 abort 旧的
        }

        current_handle_ = pending_handle_;
        pending_handle_.reset();

        return current_handle_->get_goal();
    }

    /**
     * @brief 标记当前任务成功
     */
    void succeeded_current(typename std::shared_ptr<typename ActionT::Result> result =
                           std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ && current_handle_->is_active()) {
            current_handle_->succeed(result);
        }
    }

    /**
     * @brief 终止所有任务 (用于 Cancel 或 Shutdown)
     */
    void terminate_all(typename std::shared_ptr<typename ActionT::Result> result =
                       std::make_shared<typename ActionT::Result>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ && current_handle_->is_active()) {
            current_handle_->abort(result);
        }
        if (pending_handle_ && pending_handle_->is_active()) {
            pending_handle_->abort(result);
        }
        pending_handle_.reset();
    }

    /**
     * @brief 发布反馈
     */
    void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ && current_handle_->is_active()) {
            current_handle_->publish_feedback(feedback);
        }
    }

  protected:
    // ========================================================================
    // 内部 ROS Action 回调 (Internal Callbacks)
    // ========================================================================

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const typename ActionT::Goal> /*goal*/)
    {
        // 逻辑：总是接受新目标。FineNav 的策略是"喜新厌旧"，不拒绝请求。
        // 可以在这里加一个 is_server_active 检查，如果需要支持停机维护的话。
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        // 逻辑：总是允许取消。
        if (!handle->is_active()) {
            return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief 核心状态机入口：处理已被 ROS 接受的目标
     */
    void handle_accepted(const std::shared_ptr<GoalHandle> handle)
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        // If current goal is active, put new goal into pending slot
        if (current_handle_ && current_handle_->is_active()) {
            RCLCPP_INFO(node_->get_logger(), "[%s] New goal received while busy. Storing in pending slot.", action_name_.c_str());

            // If there's already a pending goal, abort it
            if (pending_handle_ && pending_handle_->is_active()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Overwriting previous pending goal.", action_name_.c_str());
                auto result = std::make_shared<typename ActionT::Result>();
                pending_handle_->abort(result);
            }
            pending_handle_ = handle;
        }
        // Else, start executing the new goal immediately
        else {
            RCLCPP_INFO(node_->get_logger(), "[%s] New goal received. Starting execution.", action_name_.c_str());
            current_handle_ = handle;

            if (worker_thread_.joinable()) { // Expeted that thread is not running
                worker_thread_.join();
            }
            stop_execution_ = false; // TODO: 将 worker_thread_ 改为常驻线程，结合条件变量管理
            worker_thread_ = std::thread(std::bind(&SimpleActionServer::work, this));
        }
    }

    /**
     * @brief 工作线程主循环 (Worker Loop)
     * 逻辑：这个循环保证了任务的连续执行。只要还有 Pending 任务，线程就不会退出。
     */
    void work()
    {
        while (rclcpp::ok() && !stop_execution_) {
            // Execute the current goal's callback
            try {
                if (current_handle_ && current_handle_->is_active()) {
                    execute_callback_();
                }
            } catch (const std::exception & ex) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Exception in execution callback: %s", action_name_.c_str(), ex.what());
                terminate_all();
                return;
            }

            std::lock_guard<std::recursive_mutex> lock(update_mutex_);

            // If current goal is still active, but not set to succeeded, abort it
            if (current_handle_ && current_handle_->is_active()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Goal finished but result not set. Aborting.", action_name_.c_str());
                current_handle_->abort(std::make_shared<typename ActionT::Result>());
            }

            // If there's a pending goal, promote it to current, and continue loop
            if (pending_handle_) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Processing pending goal immediately.", action_name_.c_str());
                current_handle_ = pending_handle_;
                pending_handle_.reset();
            } else {
                break;
            }
        }
    }

  private:
    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    ExecuteCallback execute_callback_;

    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

    mutable std::recursive_mutex update_mutex_;

    std::shared_ptr<GoalHandle> current_handle_; // need mutex
    std::shared_ptr<GoalHandle> pending_handle_; // need mutex

    std::thread worker_thread_;
    std::atomic<bool> stop_execution_{false};
};

} // namespace finenav
```

---

### 📄 finenav_util/package.xml

```xml
<?xml version="1.0"?>
<package format="3">
    <name>finenav_util</name>
    <version>0.1.0</version>
    <description>Rolling Grid Map used in FineNav</description>

    <maintainer email="wangjinghui@sjtu.edu.cn">Jinghui Wang</maintainer>
    <license>MPL 2.0</license>

    <buildtool_depend>ament_cmake_auto</buildtool_depend>

    <depend>rclcpp</depend>
    <depend>sensor_msgs</depend>
    <depend>pcl_ros</depend>
    <depend>pcl_conversions</depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
</package>
```

---

### 📄 finenav_util/src/cloud_publish_helper.cpp

```cpp
// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_util/cloud_publish_helper.hpp"

namespace finenav_utils {

void CloudPublishHelper::configure(const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>& pub,
                    bool is_dense, const std::string& frame_id) {
    pub_ = pub;
    cloud.is_dense = is_dense;
    cloud.header.frame_id = frame_id;
};

void CloudPublishHelper::addPoint(const float& x, const float& y, const float& z,
                  const std::array<uint8_t, 3>& rgb) {
    pcl::PointXYZRGB point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.r = rgb[0];
    point.g = rgb[1];
    point.b = rgb[2];
    cloud.points.push_back(point);
}


void CloudPublishHelper::publish(rclcpp::Time time) {
    if (!pub_) return;
    cloud.width = cloud.points.size();
    cloud.height = 1;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = time;
    pub_->publish(cloud_msg);

    cloud.clear();
}

void CloudPublishHelper::reserve(size_t n) {
    cloud.points.reserve(n);
}

bool CloudPublishHelper::hasSubscribers() const {
    return pub_ && pub_->get_subscription_count() > 0;
}



} // namespace finenav_utils

```

---

### 📄 finenav_util/src/node_thread.cpp

```cpp
#include "finenav_util/node_thread.hpp"

namespace finenav {

NodeThread::NodeThread(rclcpp::Node::SharedPtr node) : node_(node) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_ = std::make_unique<std::thread>([this]() {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
    });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor) : executor_(executor) {
    thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

NodeThread::~NodeThread() {
    executor_->cancel();
    thread_->join();
}

}  // namespace finenav
```

---

