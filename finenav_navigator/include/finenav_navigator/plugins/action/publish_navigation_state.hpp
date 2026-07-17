// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

#include <std_msgs/msg/string.hpp>

#include "finenav_navigator/thirdparty/bt_topic_pub_node.hpp"

namespace finenav {

/**
 * @brief PublishNavigationState — 向指定话题发布 std_msgs::msg::String 类型的导航状态消息。
 *
 * 作为 BT::RosTopicPubNode<String> 的同步动作节点，tick 一次即发布并立即返回 SUCCESS。
 * 典型用途：在行为树的关键节点（空闲、导航中、到达目标、避障阻塞等）发布当前状态，
 * 供外部监控/可视化模块订阅。
 *
 * 输入端口:
 *   - topic_name (string, 默认 "/navigation_state"): 要发布的话题名称
 *   - state (string): 要发布的状态字符串，例如 "IDLE", "NAVIGATING", "GOAL_REACHED" 等
 *
 * Usage in XML:
 * @code
 * <PublishNavigationState topic_name="/navigation_state" state="IDLE"/>
 * <PublishNavigationState topic_name="/navigation_state" state="GOAL_REACHED"/>
 * <PublishNavigationState topic_name="/navigation_state" state="NAVIGATING"/>
 * <PublishNavigationState topic_name="/navigation_state" state="MPPI_OBSTACLE_BLOCKED"/>
 * @endcode
 */
class PublishNavigationState : public BT::RosTopicPubNode<std_msgs::msg::String>
{
public:
    PublishNavigationState(const std::string& name,
                           const BT::NodeConfig& conf,
                           const BT::RosNodeParams& params)
        : BT::RosTopicPubNode<std_msgs::msg::String>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("state", "IDLE", "Navigation state string to publish"),
        });
    }

    bool setMessage(std_msgs::msg::String& msg) override
    {
        getInput("state", state_);
        msg.data = state_;
        return true;
    }

private:
    std::string state_;
};

}  // namespace finenav
