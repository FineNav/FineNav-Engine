# 4. Assemble a control pipeline

With the Map, Planners, and WorldViews selected, the application only needs to
load the plugin package, create the layers, and register their execution order.

## Example application

This is `finenav_example/getting_started/hello_finenav.cpp`:

```cpp
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
    engine->createControlLayer(
        "dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
    engine->createControlLayer(
        "dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
    engine->registerControlBranch("navigate", {"line_layer", "follow_layer"});

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
```

## Data flow

```mermaid
flowchart LR
    G[Navigation goal] --> L["line_layer · event driven"]
    L -->|Pose path| F["follow_layer · 10 Hz"]
    F --> C[/cmd_vel/]
```

| Call | What it configures |
| --- | --- |
| `setDefaultBehaviorTree("GettingStartedNavigate")` | The task tree used for goals when no tree ID is supplied. |
| `loadAlgoPlugins({"finenav_example"})` | Discovers the installed Map, Planner, and WorldView libraries in the package. |
| `createControlLayer(..., "line_layer", EventDrivenPolicy{})` | Generates a path when a new upstream reference arrives. |
| `createControlLayer(..., "follow_layer", PeriodicPolicy{10.0})` | Follows the latest path using current robot state at a target 10 Hz. |
| `registerControlBranch("navigate", {"line_layer", "follow_layer"})` | Connects the layers in that order under branch name `navigate`. |

The first layer consumes the navigation goal; the second consumes the first
layer's pose path. Do not add `_root` to the list. Registering a branch defines
the connection; the Behavior Tree activates it with `ExecuteControlBranch`.

These assembly methods return `bool`. The minimal example omits error handling;
in your application, check their return values and stop initialization on failure.

## Parameter configuration

`hello_finenav.launch.py` loads this runtime configuration:

```yaml
/hello_finenav:
  ros__parameters:
    use_sim_time: true

/hello_finenav/finenav_navigator:
  ros__parameters:
    behavior_trees:
      - "finenav_example/getting_started/behavior_trees"

/hello_finenav/finenav_localizer:
  ros__parameters:
    pose_topic: ""
    pose_smoothing_time: 0.0
    initial_pose_topic: "/initialpose"
    twist_topic: ""

    odom_topic: "/odom"
    odom_use_twist: true

    state_history_duration: 5.0

    force_2d_mode: true
    publish_tf: false
    publish_odom: true

/hello_finenav/dummy_map:
  ros__parameters:
    dummy_map:
      window_size: 50.0

/hello_finenav/line_layer:
  ros__parameters:
    dummy_line_planner:
      path_resolution: 0.2

/hello_finenav/follow_layer:
  ros__parameters:
    dummy_follow_planner:
      linear_speed: 0.5
      angular_speed: 1.0
      lookahead_distance: 1.0
```

`/hello_finenav/line_layer` and `/hello_finenav/follow_layer` come from the layer
names in C++; `dummy_line_planner` and `dummy_follow_planner` are the parameter
namespaces. If you rename a layer, update its YAML node path too.
The localizer reads `/odom`, while the default chassis interface publishes
`geometry_msgs/msg/Twist` on `/cmd_vel`.

For example, while the application is running, reduce its follower's configured
forward speed:

```bash
ros2 param set /hello_finenav/follow_layer dummy_follow_planner.linear_speed 0.3
```

Planner speed settings are dynamically configurable. C++ assembly changes need
a rebuild and restart. The YAML file selects the installed behavior-tree
directory, whose tree ID must match the C++ default.

Source: [application](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/getting_started/hello_finenav.cpp),
[runtime YAML](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/config/hello_finenav.yaml),
[public API](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_engine_cpp/include/finenav_engine_cpp/finenav_engine.hpp).

Continue with [5. Configure a Behavior Tree](behavior-tree.md).
