# 1. Run the example

Complete [Installation](installation.md), then use one launch file to start the
simulator, navigation application, and RViz.

## Launch

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

Press **Play** in Gazebo. The launch file uses Gazebo odometry, loads
`hello_finenav.yaml` and `hello_finenav.rviz`, and uses simulation time.
RViz is configured with the `map` frame, robot model, TF, odometry, and goal tools.

## Send a goal

1. Use RViz's **2D Pose Estimate** to set an initial pose consistent with the simulated robot.
2. Use **2D Goal Pose** to select a nearby goal.

Choose a short unobstructed straight path and observe the robot moving.
The example does not publish an occupancy map.

## Launch arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `world_name` | `flat_small_scale_maze.world` | Simulator world. |
| `params_file` | `config/hello_finenav.yaml` | Navigation application parameters. |
| `use_rviz` | `true` | Whether to start RViz. |

Launch without RViz:

```bash
ros2 launch finenav_example hello_finenav.launch.py use_rviz:=false
```

## Output checks

In another sourced terminal, run these commands one at a time:

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel
```

`/clock` should advance, `/odom` should update, and `/cmd_vel` should contain
commands while the branch is running. The follower runs at a configured 10 Hz;
this is its target rate, not a timing guarantee.

| Symptom | First check |
| --- | --- |
| Package or executable not found | Build succeeded and `install/setup.bash` was sourced. |
| Robot remains still | Gazebo is playing, `/odom` updates, and `/cmd_vel` has commands. |
| Tree or algorithm name not found | Installed behavior-tree files and plugin resource exist; inspect startup errors. |
| Robot approaches the goal but task does not finish | Read the completion criteria in [step 5](behavior-tree.md); the dummy follower has no terminal braking or final-heading controller. |

`DummyMap::isOccupied()` always returns false. The robot is not planning around
the maze walls. Inspect the [example application](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/hello_finenav.cpp)
and [launch configuration](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/config/hello_finenav.yaml).

Continue with [2. Choose Map and Planner](choose-algorithms.md).
