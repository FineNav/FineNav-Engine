# Maps

Map plugins represent the environment and expose data used by WorldViews.
Choose a Map according to sensor inputs, environment representation, and the
queries required by the selected Planners.

## Available entries

| Plugin | Purpose | Repository |
| --- | --- | --- |
| `dummy_map` | Minimal example Map for plugin integration. | [FineNav-Engine](https://github.com/FineNav/FineNav-Engine/tree/dev/finenav_example/create_custom_map) |

## Dummy Map

- **Package:** `finenav_example`
- **Registered name:** `dummy_map`
- **Example pairings:** `dummy_line_planner` through `dummy_line_world_view`, and
  `dummy_follow_planner` through `dummy_follow_world_view`

This Map exposes an occupancy query that always returns false and has no
observation sources. Both example WorldViews resolve to the same Map type, so
FineNavEngine reuses one `DummyMap` server for both control layers.

`DummyMap` does not contain obstacle data. Use a Map with an appropriate
environment representation for navigation that requires obstacle information.

For installation and execution, follow the [Quickstart](../quickstart.md). To
connect another Map to a Planner, see [WorldView integration](../worldview.md).
