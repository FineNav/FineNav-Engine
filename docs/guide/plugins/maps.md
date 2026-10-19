# finenav_maps

Map components are maintained in
[FineNav/finenav_map](https://github.com/FineNav/finenav_map).

## Download

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
```

## Component catalogue

`stvl_map` is a rolling spatio-temporal voxel Map updated with
`sensor_msgs/msg/PointCloud2` point clouds.
Its registered Map name and parameter namespace are both `stvl_map`.

It accepts clouds in the map frame or in
the configured input frame (default `base_link`), and removes stale occupied
voxels according to its decay setting. See its
[README and parameter schema](https://github.com/FineNav/finenav_map/tree/main/stvl_map).

## Engine integration status

The `stvl_map` CMake file does not yet publish the ament resource
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
