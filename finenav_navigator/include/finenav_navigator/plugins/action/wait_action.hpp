// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace finenav::navigator {

/**
 * @brief Wait — 阻塞等待指定时长后返回 SUCCESS。
 *
 * 作为 BT::StatefulActionNode，首次 tick 记录起始时间并返回 RUNNING，
 * 后续每次 tick 检查是否已过 duration_ms，到时返回 SUCCESS。
 * halt() 重置内部计时器。
 *
 * 输入端口:
 *   - duration_ms (unsigned, 默认 1000): 等待时长，单位毫秒
 *
 * Usage in XML:
 * @code
 * <Wait duration_ms="2000"/>
 * @endcode
 */
class WaitAction : public BT::StatefulActionNode {
public:
    WaitAction(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<unsigned>("duration_ms", 1000, "Wait duration in milliseconds"),
        };
    }

protected:
    BT::NodeStatus onStart() override {
        getInput("duration_ms", duration_ms_);
        start_time_ = std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(now - start_time_).count();
        if (elapsed_ms >= static_cast<double>(duration_ms_)) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        duration_ms_ = 1000;
        start_time_  = std::chrono::steady_clock::time_point{};
    }

private:
    unsigned duration_ms_{1000};
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace finenav::navigator
