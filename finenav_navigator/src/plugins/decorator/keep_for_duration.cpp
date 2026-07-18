// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/decorator/keep_for_duration.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace finenav {

KeepForDuration::KeepForDuration(const std::string& name, const BT::NodeConfig& config)
    : BT::DecoratorNode(name, config) {}

BT::NodeStatus KeepForDuration::tick() {
    double duration = 10.0;
    getInput("duration_sec", duration);

    const auto child_status = child_node_->executeTick();

    if (child_status == BT::NodeStatus::SUCCESS) {
        const auto now = std::chrono::steady_clock::now();
        if (!timer_running_) {
            timer_start_  = now;
            timer_running_ = true;
        }
        const double elapsed =
            std::chrono::duration<double>(now - timer_start_).count();
        if (elapsed >= duration) {
            timer_running_ = false;
            // 重置子节点，以便下次被 tick 时重新开始
            haltChild();
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    // 子节点 FAILURE 或 RUNNING → 计时中断
    timer_running_ = false;
    return child_status;
}

void KeepForDuration::halt() {
    timer_running_ = false;
    DecoratorNode::halt();
}

}  // namespace finenav

// Pure BT plugin (no ROS node params needed)
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<finenav::KeepForDuration>("KeepForDuration");
}
