// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>

#include "finenav_navigator/plugins/decorator/dynamic_goal_follower.hpp"

namespace finenav
{

DynamicGoalFollower::DynamicGoalFollower(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::DecoratorNode(name, conf),
  node_(params.nh),
  last_msg_time_(0, 0, RCL_ROS_TIME)
{
}

void DynamicGoalFollower::initialize()
{
  // 订阅只创建一次，避免 halt/retry 循环中重复订阅导致旧消息队列丢失
  if (sub_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    throw BT::RuntimeError("DynamicGoalFollower: ROS node has gone out of scope.");
  }

  std::string topic_name = "/target_pose";
  getInput("topic_name", topic_name);

  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(
    callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_name,
    rclcpp::QoS(1),
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      onTargetPose(msg);
    },
    options);
}

BT::NodeStatus DynamicGoalFollower::tick()
{
  // 首次进入或重新激活时初始化订阅
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  // 处理积压的 ROS 回调，获取最新目标位姿
  callback_executor_.spin_some();

  auto node = node_.lock();
  if (!node) {
    return BT::NodeStatus::FAILURE;
  }

  double timeout_sec = 1.0;
  getInput("timeout_sec", timeout_sec);

  // 1. 检查超时或是否接收到目标 (Timeout / Availability check)
  if (!last_msg_received_ || (node->now() - last_msg_time_).seconds() > timeout_sec) {
    RCLCPP_DEBUG(node->get_logger(), "DynamicGoalFollower: No active target pose received within timeout.");
    return BT::NodeStatus::FAILURE;
  }

  // 从 blackboard 获取机器人当前位姿
  geometry_msgs::msg::Pose robot_pose;
  if (!getInput("robot_pose", robot_pose)) {
    RCLCPP_WARN(node->get_logger(), "DynamicGoalFollower: robot_pose not available on blackboard.");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 计算并检查距离阈值 (Range checks)
  const double dx = last_msg_.pose.position.x - robot_pose.position.x;
  const double dy = last_msg_.pose.position.y - robot_pose.position.y;
  const double dist = std::hypot(dx, dy);

  double min_distance = 0.5;
  double max_distance = 5.0;
  getInput("min_distance", min_distance);
  getInput("max_distance", max_distance);

  // 目标过远：停止跟随并上报 FAILURE (让行为树可以走 fallback 恢复)
  if (dist > max_distance) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
      "DynamicGoalFollower: Target is too far (%.2fm > %.2fm). Stopping follow (Failure).", dist, max_distance);
    return BT::NodeStatus::FAILURE;
  }

  // 目标过近：视为抵达，停止跟随并上报 SUCCESS
  if (dist < min_distance) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
      "DynamicGoalFollower: Target is within reach (%.2fm < %.2fm). Stopping follow (Success).", dist, min_distance);
    return BT::NodeStatus::SUCCESS;
  }

  // 3. 计算跟随目标
  const auto follow_goal = computeFollowGoal(robot_pose, last_msg_.pose);

  if (shouldUpdate(follow_goal)) {
    setOutput("goal", follow_goal);
    last_goal_ = follow_goal;
    has_last_goal_ = true;
  }

  // 透传子节点状态
  setStatus(BT::NodeStatus::RUNNING);
  const BT::NodeStatus child_status = child_node_->executeTick();
  return child_status;
}

void DynamicGoalFollower::halt()
{
  // 保留订阅和 last_msg_received_ 状态：halt/retry 循环中不能丢弃已收到的消息，
  // 否则新 tick 的 timeout 检查会因为没有"新"消息而立即返回 FAILURE，造成死循环。
  has_last_goal_ = false;
  DecoratorNode::halt();
}

geometry_msgs::msg::Pose DynamicGoalFollower::computeFollowGoal(
  const geometry_msgs::msg::Pose & robot_pose,
  const geometry_msgs::msg::Pose & target_pose) const
{
  auto goal = target_pose;

  double follow_distance = 0.0;
  getInput("follow_distance", follow_distance);
  follow_distance = std::max(0.0, follow_distance);

  const double dx = target_pose.position.x - robot_pose.position.x;
  const double dy = target_pose.position.y - robot_pose.position.y;
  const double dist = std::hypot(dx, dy);

  // 如果设置了跟随距离且距离大于阈值，将目标沿 robot→target 方向回退
  if (follow_distance > 0.0 && dist > follow_distance) {
    const double scale = (dist - follow_distance) / dist;
    goal.position.x = robot_pose.position.x + dx * scale;
    goal.position.y = robot_pose.position.y + dy * scale;
  }

  // 让目标朝向指向原目标方向
  const double yaw = std::atan2(dy, dx);
  goal.orientation.w = std::cos(0.5 * yaw);
  goal.orientation.x = 0.0;
  goal.orientation.y = 0.0;
  goal.orientation.z = std::sin(0.5 * yaw);

  return goal;
}

bool DynamicGoalFollower::shouldUpdate(
  const geometry_msgs::msg::Pose & new_goal) const
{
  if (!has_last_goal_) {
    return true;
  }

  double update_distance = 0.05;
  getInput("update_distance", update_distance);
  update_distance = std::max(0.0, update_distance);

  const double dx = new_goal.position.x - last_goal_.position.x;
  const double dy = new_goal.position.y - last_goal_.position.y;
  const double dz = new_goal.position.z - last_goal_.position.z;
  return (dx * dx + dy * dy + dz * dz) >= update_distance * update_distance;
}

void DynamicGoalFollower::onTargetPose(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  last_msg_ = *msg;
  last_msg_received_ = true;
  if (auto node = node_.lock()) {
    last_msg_time_ = node->now();
  }
}

}  // namespace finenav

#include "finenav_navigator/thirdparty/plugins.hpp"
CreateRosNodePlugin(finenav::DynamicGoalFollower, "DynamicGoalFollower");