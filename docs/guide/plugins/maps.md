# finenav_maps

Map components are maintained in
[FineNav/finenav_map](https://github.com/FineNav/finenav_map).
The site section uses the plural name `finenav_maps`; the repository name is singular.

## Component catalogue

The repository's `main` branch at revision `e5abf6a` contains `stvl_map`,
a rolling spatio-temporal voxel Map fed by `sensor_msgs/msg/PointCloud2`.
Its registered Map name and parameter namespace are both `stvl_map`.

The inspected implementation accepts clouds already in the map frame or in
the configured input frame (default `base_link`), and removes stale occupied
voxels according to its decay setting. See its
[README and parameter schema](https://github.com/FineNav/finenav_map/tree/e5abf6a/stvl_map).
Other feature-branch experiments are not listed as default-branch components.

## Engine integration status

The inspected `stvl_map` CMake file does not yet publish the ament resource
required by the new `loadAlgoPlugins({"package_name"})` API.
Add plugin discovery registration and a WorldView for the chosen Planner before
using it with the current Engine. The linked README still shows older engine
assembly calls; follow the current [WorldView](../worldview.md) and
[pipeline](../cpp-application.md) tutorials for the application API.

## Example Map

Getting Started uses `dummy_map` from ROS package `finenav_example`.
It has no observation sources, and `isOccupied()` always returns false.
Both example WorldViews share its one Map server. It is useful for integration
exercises, not obstacle-aware navigation.

The schema exposes `window_size` as read-only, but the current dummy implementation
keeps fixed query bounds; do not use that setting as a demonstration of a resized map.

For your own representation and sensor inputs, continue with
[Custom Map](../custom-map.md).
