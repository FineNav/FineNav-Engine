# FineNav-Engine

**Version: 0.5.0**

FineNav-Engine is a ROS 2 C++ framework for assembling layered navigation systems.
Choose Map and Planner plugins, connect them through planner-specific WorldViews,
and assemble control layers with the engine API. Behavior trees activate
registered control branches and coordinate navigation, stopping, and recovery.

## Build

Use Ubuntu 22.04 with ROS 2 Humble Desktop installed. Run these commands in Bash:

```bash
sudo apt update
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep libbenchmark-dev
source /opt/ros/humble/setup.bash

mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone --branch dev https://github.com/FineNav/FineNav-Engine.git src/FineNav-Engine
```

Initialize rosdep once per machine, then install the manifest dependencies:

```bash
sudo rosdep init  # Skip if already initialized.
rosdep update
cd ~/finenav_ws
rosdep install --from-paths src/FineNav-Engine --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-up-to finenav_example finenav_simulator
source install/setup.bash
```

## Run the example

Source ROS and the workspace in each terminal. Start the simulator:

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
ros2 launch finenav_simulator finenav_simulator.launch.py lio_type:=gazebo_odom
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

See the [simulation tutorial](finenav_example/getting_started/README.md) for
goal input, configuration, and example limitations. The example uses the plugins
in this repository; additional Map and Planner packages can be integrated separately.

## Packages

| Package | Purpose |
| --- | --- |
| [finenav_engine](finenav_engine/) | Engine metapackage. |
| [finenav_engine_cpp](finenav_engine_cpp/) | C++ application API and runtime assembly. |
| [finenav_core](finenav_core/) | Plugin contracts, maps, control layers, and robot state interfaces. |
| [finenav_navigator](finenav_navigator/) | Behavior trees and control branch execution. |
| [finenav_localizer](finenav_localizer/) | Robot state from pose, velocity, and odometry observations. |
| [finenav_interfaces](finenav_interfaces/) | ROS messages and navigation action. |
| [finenav_example](finenav_example/) | Sample plugins and navigation application. |
| [finenav_simulator](finenav_simulator/README.md) | Gazebo simulation, sensor bridges, and helper nodes. |
| [finenav_visualizer](finenav_visualizer/) | Marker and point-cloud visualization channels. |
| [finenav_common](finenav_common/) | Shared geometry, frames, parameters, and registries. |
| [finenav_util](finenav_util/) | ROS threading, action server, logging, and timing utilities. |
| [finenav_cmake](finenav_cmake/) | Plugin discovery registration and parameter build helpers. |

## Documentation

- [Developer documentation](https://finenav.github.io/finenav-engine/)
- [FineNav organization](https://github.com/FineNav)
