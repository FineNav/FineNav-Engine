# FineNav Plugins

Find Map and Planner components to use with FineNav-Engine. Plugins may be
included in the engine repository or distributed in an integration repository.

## Browse plugins

| Category | What to look for |
| --- | --- |
| [Maps](maps.md) | Environment representations and the queries they provide. |
| [Planners](planners.md) | Planning algorithms, supported inputs, and navigation outputs. |

The catalogue currently documents the built-in example plugins. Integration
repositories provide their own installation and compatibility information.

## Use a plugin

1. Choose Planners that produce the required pipeline data.
2. Choose Maps that provide the required environment data.
3. Use supplied WorldViews or [write one](../worldview.md) for each Planner and Map pairing.
4. Load the plugin package and assemble the WorldViews in a
   [C++ application](../cpp-application.md).

The built-in example combines `dummy_line_planner`, `dummy_follow_planner`, and
`dummy_map` through `dummy_line_world_view` and `dummy_follow_world_view`. Follow
the [Quickstart](../quickstart.md) to build and run the layered pipeline.
