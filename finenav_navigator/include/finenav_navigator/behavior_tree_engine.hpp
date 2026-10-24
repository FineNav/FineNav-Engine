// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace finenav::navigator {

class BehaviorTreeEngine {
  public:
    BehaviorTreeEngine();

    void registerPlugins(const std::vector<std::string>& plugins);
    void registerBehaviorTrees(const std::vector<std::string>& tree_dirs);

    bool loadPlugin(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path);
    bool loadBehaviorTree(BT::BehaviorTreeFactory& factory, const std::filesystem::path& file_path);

    BT::BehaviorTreeFactory& factory() { return factory_; }

  private:
    static std::pair<std::string, std::string> parseParamPath(const std::string& input);

    BT::BehaviorTreeFactory factory_;
};

}  // namespace finenav::navigator
