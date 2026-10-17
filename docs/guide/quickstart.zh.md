# 1. 运行示例

完成[如何安装](installation.md)后，运行原始 `finenav_example`，检查机器人运动和速度输出。

## 启动仿真：终端 1

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=gazebo_odom use_sim_time:=true \
  world_name:=flat_small_scale_maze.world publish_map_tf:=true
```

在 Gazebo 中点击 **Play**。`gazebo_odom` 表示使用仿真里程计，避开 Fast-LIO 定位链。
此模式的 map 到 odom 变换由仿真辅助节点提供。

## 启动导航应用：终端 2

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

launch 文件加载 `finenav_example/config/hello_finenav.yaml`，启动
`getting_started/hello_finenav.cpp`。等待 `FineNav-Engine ready.`，并检查它之前是否有插件加载错误；
这行提示本身不能证明组件已经成功组装。

## 发送目标：终端 3

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

1. 将 **Fixed Frame** 设为 `map`。
2. 按需添加 **RobotModel**、**TF**，通过订阅 `/odom` 的 **Odometry** 显示确定机器人位置。
3. 使用 **2D Pose Estimate** 向 `/initialpose` 发布初始位姿，与仿真中机器人的实际位置保持一致。
4. 使用 **2D Goal Pose** 向 `/goal_pose` 发送附近的目标；工具栏没有该项时，通过 **+** 添加。

选择一段无遮挡的直线路径。预期结果：机器人沿路径开始运动。
本示例不发布占据地图，因此 RViz 的 Map 显示为空并不代表启动失败。

## 输出检查

在另一个已加载环境的终端中，分别运行：

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel
```

`/clock` 应持续推进，`/odom` 应更新，分支运行时 `/cmd_vel` 应有速度指令。
跟踪层配置为 10 Hz，这是目标执行频率，并非实时性保证。

| 现象 | 优先检查 |
| --- | --- |
| 找不到包或可执行文件 | 编译是否成功，是否加载了 `install/setup.bash`。 |
| 机器人不动 | Gazebo 是否开始播放，`/odom` 是否更新，`/cmd_vel` 是否输出。 |
| 找不到行为树或算法名称 | 安装后的行为树文件和插件资源是否存在，启动日志是否报错。 |
| 接近目标但任务不结束 | 查看[第 5 步](behavior-tree.md)的完成条件；Dummy 跟踪器没有到点减速和终点朝向控制。 |

`DummyMap::isOccupied()` 始终返回 false，因此此示例不会绕开迷宫墙壁。
源码入口：[示例应用](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/getting_started/hello_finenav.cpp)、
[启动配置](https://github.com/FineNav/FineNav-Engine/blob/main/finenav_example/config/hello_finenav.yaml)。

下一步：[2. 选择 Map 和 Planner](choose-algorithms.md)。
