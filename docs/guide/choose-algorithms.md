# 2. Choose Map and Planner

Start from the inputs and outputs your application needs. In this example, a goal
becomes a pose path, and that path becomes velocity commands.

## Example components

| Role | Implementation / registered name | Purpose |
| --- | --- | --- |
| Map | `DummyMap` / `dummy_map` | Supplies a minimal occupancy query; every position is unoccupied. |
| Path Planner | `DummyLinePlanner` / `dummy_line_planner` | Current robot pose + goal → straight pose path. |
| Tracking Planner | `DummyFollowPlanner` / `dummy_follow_planner` | Robot state + pose path → velocity command. |
| First adapter | `DummyLineWorldView` / `dummy_line_world_view` | Connects the Map to the line Planner. |
| Second adapter | `DummyFollowWorldView` / `dummy_follow_world_view` | Connects the same Map to the follower. |

All five components are supplied by ROS package `finenav_example`. They do not
come from the separate Map and Planner repositories.

## Component configuration

```cpp
engine->loadAlgoPlugins({"finenav_example"});
engine->createControlLayer(
    "dummy_line_world_view", "line_layer", core::EventDrivenPolicy{});
engine->createControlLayer(
    "dummy_follow_world_view", "follow_layer", core::PeriodicPolicy{10.0});
```

`loadAlgoPlugins()` takes **ROS package names**, without `/lib`. Each package
must publish its plugin libraries with `finenav_register_algo_plugin()`.
For these Map-reading Planners, `createControlLayer()` selects a **WorldView
registration**, which already binds the Map and Planner types. It does not
take two independent Map/Planner names. The engine creates and shares the
required `dummy_map` server automatically.

## Integration checklist

| Item | Requirement |
| --- | --- |
| Environment | The Map represents the environment and accepts the sensor inputs. |
| Map → Planner | The WorldView implements every query required by the Planner's `IWorldView`. |
| Planner → next layer | The output contains the pose / velocity fields required by the next layer. |
| Final output | The final output matches the chassis interface; this example uses `/cmd_vel`. |
| Plugin packaging | The package declares the resources required by the current Engine's ament discovery API. |

For alternatives, see [finenav_planners](plugins/planners.md) and
[finenav_maps](plugins/maps.md). Cloning a repository alone does not create
the WorldView that connects a chosen Map and Planner.

Source: [line Planner](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_planner/include/dummy_line_planner.hpp),
[follower](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_planner/include/dummy_follow_planner.hpp),
[Map](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/create_custom_map/include/dummy_map.hpp).

Continue with [3. Write a WorldView](worldview.md).
