// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine_cpp/finenav_engine.hpp"

using namespace finenav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    engine->setDefaultBehaviorTree("GettingStartedNavigate");

    engine->loadAlgoPlugins({"finenav_example"});
    engine->createControlLayer("dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
    engine->createControlLayer("dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
    engine->registerControlBranch("navigate", {"line_layer", "follow_layer"});

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
