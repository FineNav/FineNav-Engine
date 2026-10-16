# Configure Your Application

Use `FineNavEngine` to load algorithm packages, assemble control layers, and
register an ordered control branch.

## Assemble navigation

The example generates a pose path on each new goal, then follows that path at
10 Hz:

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine_cpp/finenav_engine.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    engine->setDefaultBehaviorTree("GettingStartedNavigate");

    engine->loadAlgoPlugins({"finenav_example"});
    engine->createControlLayer(
        "dummy_line_world_view", "line_layer",
        finenav::core::EventDrivenPolicy{});
    engine->createControlLayer(
        "dummy_follow_world_view", "follow_layer",
        finenav::core::PeriodicPolicy{10.0});
    engine->registerControlBranch(
        "navigate", {"line_layer", "follow_layer"});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

`loadAlgoPlugins()` accepts package names. Each package publishes its algorithm
libraries in the `finenav_algo_plugins` ament resource. A WorldView registration
selects its Planner and Map types, so the application only names the WorldView
and layer. Layers that resolve to the same Map type share one map server.

Keep the branch name `navigate` when using `GettingStartedNavigate`, because the
behavior tree activates that name with `ExecuteControlBranch`.

## Set runtime parameters

Use the example
[parameter file](https://github.com/FineNav/FineNav-Engine/blob/dev/finenav_example/config/hello_finenav.yaml)
as a starting point:

| Setting | What to configure |
| --- | --- |
| `behavior_trees` | Include the directory containing `GettingStartedNavigate`. |
| Localization topics | Connect the robot's odometry or absolute pose input. |
| `use_sim_time` | Enable for simulation; disable for a physical robot. |
| Component parameters | Set values from each selected plugin's YAML schema. |

The example uses `/hello_finenav` as the host node and places framework nodes
below that namespace. If the robot uses frames other than `map` and
`base_link`, call `setFrameNames()` before creating maps or layers.

## Send a goal

Use RViz's `/goal_pose` input or invoke the C++ API:

```cpp
geometry_msgs::msg::Pose goal;
goal.position.x = 2.0;
goal.position.y = 1.0;
goal.orientation.w = 1.0;
engine->navigateTo({goal});

engine->cancelNavigation();
```

For executable build configuration, use the
[getting-started CMake example](https://github.com/FineNav/FineNav-Engine/blob/dev/finenav_example/getting_started/CMakeLists.txt).
