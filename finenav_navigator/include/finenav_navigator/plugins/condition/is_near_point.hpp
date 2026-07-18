// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include "behaviortree_cpp/condition_node.h"

namespace finenav {

/**
 * @brief IsNearPoint — 检查两个位姿点之间的 2D 距离是否小于指定阈值。
 *
 * 适用于"到达终点附近"区域判定，配合 KeepForDuration 实现
 * "在终点附近持续一段时间后认为完成导航"的逻辑。
 *
 * Blackboard 输入:
 *   - point_a (geometry_msgs::msg::Pose): 第一个位姿点（通常是 robot_pose）
 *   - point_b (geometry_msgs::msg::Pose): 第二个位姿点（通常是 goal）
 *
 * 输入端口:
 *   - distance_threshold (double, 默认 3.0 m): 距离判定阈值
 *
 * Usage in XML:
 * @code
 * <IsNearPoint point_a="{robot_pose}" point_b="{goal}" distance_threshold="3.0"/>
 * @endcode
 */
class IsNearPoint : public BT::ConditionNode {
public:
    IsNearPoint(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::Pose>("point_a",           "{robot_pose}", "First pose"),
            BT::InputPort<geometry_msgs::msg::Pose>("point_b",           "{goal}",       "Second pose"),
            BT::InputPort<double>("distance_threshold", 3.0, "Distance threshold [m]"),
        };
    }

    BT::NodeStatus tick() override {
        geometry_msgs::msg::Pose a, b;
        double threshold = 3.0;

        if (!getInput("point_a", a) || !getInput("point_b", b)) {
            return BT::NodeStatus::FAILURE;
        }
        getInput("distance_threshold", threshold);

        const double dx   = a.position.x - b.position.x;
        const double dy   = a.position.y - b.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        return (dist < threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}  // namespace finenav
