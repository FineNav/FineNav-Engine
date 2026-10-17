# 1. Run the example

Complete [Installation](installation.md) first. In this step you run the
unmodified `finenav_example` application and observe its velocity output.

## Start the simulator — terminal 1

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=gazebo_odom use_sim_time:=true \
  world_name:=flat_small_scale_maze.world publish_map_tf:=true
```

Press **Play** in Gazebo. `gazebo_odom` selects simulator odometry instead of
the Fast-LIO chain. The simulator helper provides the map-to-odom transform in this mode.

## Start the navigation application — terminal 2

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

The launch file loads `finenav_example/config/hello_finenav.yaml` and starts
`getting_started/hello_finenav.cpp`. Wait for `FineNav-Engine ready.` and inspect
earlier messages for plugin-loading errors; that line alone does not confirm successful assembly.

## Send a goal — terminal 3

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

1. Set **Fixed Frame** to `map`.
2. Add **RobotModel** and **TF** displays if needed. An **Odometry** display on `/odom` helps locate the robot.
3. Use **2D Pose Estimate** to publish an initial pose on `/initialpose`, consistent with the robot's simulated pose.
4. Use **2D Goal Pose**, or RViz's **2D Goal Pose** tool added through **+**, to publish a nearby goal on `/goal_pose`.

Choose a short unobstructed segment for this demonstration. You should see the
robot start following the straight path. This example does not publish an occupancy
map, so a blank Map display is expected.

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
the maze walls. Inspect the [example application](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/getting_started/hello_finenav.cpp)
and [launch configuration](https://github.com/FineNav/FineNav-Engine/blob/2c5bbe5/finenav_example/config/hello_finenav.yaml).

Continue with [2. Choose Map and Planner](choose-algorithms.md).
