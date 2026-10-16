# Getting Started - Simulation

Minimal FineNav-Engine navigation stack driven by a simulator.

```text
/goal_pose -> FineNavEngine -> branch "navigate"
                                  |
                       DummyLinePlanner
                       EventDrivenPolicy
                                  |
                             pose path
                                  |
                       DummyFollowPlanner
                       PeriodicPolicy{10.0}
                                  |
                             /cmd_vel

DummyLineWorldView ----+
                       +---- shared DummyMap
DummyFollowWorldView --+
```

## Prerequisites

- Ubuntu 22.04 and ROS 2 Humble
- Workspace built with `colcon build --symlink-install`
- Workspace environment sourced with `source install/setup.bash`

## Run

Start the simulator:

```bash
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=static \
  use_sim_time:=true \
  world_name:=flat_small_scale_maze.world \
  publish_map_tf:=true
```

Start FineNav-Engine:

```bash
ros2 launch finenav_example hello_finenav.launch.py
```

The launch file loads all runtime settings from `config/hello_finenav.yaml`.

Publish an initial pose with RViz, then use **2D Goal Pose** to send a goal on
`/goal_pose`. Inspect the output with:

```bash
ros2 topic echo /cmd_vel
```

## Pipeline

`hello_finenav.cpp` loads the `finenav_example` plugin package and creates two
control layers:

```cpp
engine->createControlLayer(
    "dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
engine->createControlLayer(
    "dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
engine->registerControlBranch("navigate", {"line_layer", "follow_layer"});
```

`line_layer` generates a straight pose path for each new goal. `follow_layer`
selects a fixed-distance lookahead point from that path and publishes velocity
commands at 10 Hz. FineNavEngine resolves both WorldView registrations to the
same `DummyMap` type and reuses one map server.

The `GettingStartedNavigate` behavior tree activates the `navigate` branch,
stops the chassis when the goal is reached, and restarts from the first faulted
control layer when recovery is required.

## Parameters

| Node | Parameter | Purpose |
| --- | --- | --- |
| `/hello_finenav` | `use_sim_time` | Use the simulator clock. |
| `/hello_finenav/finenav_navigator` | `behavior_trees` | Load the example behavior tree. |
| `/hello_finenav/finenav_localizer` | `odom_topic` | Read simulator odometry. |
| `/hello_finenav/finenav_localizer` | `state_history_duration` | Retain states for delayed measurements. |
| `/hello_finenav/dummy_map` | `dummy_map.window_size` | Set the map window half-extent. |
| `/hello_finenav/line_layer` | `dummy_line_planner.path_resolution` | Set the straight-path sample spacing. |
| `/hello_finenav/follow_layer` | `dummy_follow_planner.*` | Set tracking speeds and lookahead distance. |

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Robot does not move | Confirm `/cmd_vel` is published and simulation time advances. |
| No navigation tree | Confirm the workspace is sourced and `behavior_trees` resolves. |
| Plugin not registered | Check the `finenav_example` ament plugin resource after building. |
