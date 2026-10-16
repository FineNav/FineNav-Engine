# Planners

Planner plugins compute paths or motion commands. Choose and order Planners
according to the pipeline data each stage consumes and produces.

## Available entries

| Plugin | Input -> Output | Repository |
| --- | --- | --- |
| `dummy_line_planner` | Goal poses -> pose path | [FineNav-Engine](https://github.com/FineNav/FineNav-Engine/tree/dev/finenav_example/create_custom_planner) |
| `dummy_follow_planner` | Pose path -> velocity command | [FineNav-Engine](https://github.com/FineNav/FineNav-Engine/tree/dev/finenav_example/create_custom_planner) |

## Dummy Line Planner

- **Package:** `finenav_example`
- **Registered name:** `dummy_line_planner`
- **Example pairing:** `dummy_map` through `dummy_line_world_view`

This Planner samples a straight pose path from the robot's current pose to the
final reference pose. `path_resolution` controls sample spacing. The Planner
returns no result if any sampled point is not walkable.

## Dummy Follow Planner

- **Package:** `finenav_example`
- **Registered name:** `dummy_follow_planner`
- **Example pairing:** `dummy_map` through `dummy_follow_world_view`

This Planner finds the closest path sample, advances by `lookahead_distance`,
and produces linear and angular velocity commands toward that target. Its speed
parameters are hot-reloadable.

The example runs the line Planner when a new goal arrives and the follower at
10 Hz. For installation and execution, follow the [Quickstart](../quickstart.md).
To connect a different Planner to a Map, see [WorldView integration](../worldview.md).
