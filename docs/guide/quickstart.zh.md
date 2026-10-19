# 1. 运行示例

完成[如何安装](installation.md)后，通过一个 launch 文件启动仿真、导航应用和 RViz。

## 启动

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch finenav_example hello_finenav.launch.py
```

启动后，在 Gazebo 中点击 **Play**。launch 文件使用 Gazebo 里程计，
加载 `hello_finenav.yaml` 和 `hello_finenav.rviz`，并统一使用仿真时间。
RViz 已设置 `map` 坐标系、机器人模型、TF、里程计和目标点工具。

## 发送目标

1. 使用 RViz 的 **2D Pose Estimate** 设置初始位姿，与仿真中机器人位置保持一致。
2. 使用 **2D Goal Pose** 在机器人附近设置目标点。

选择一段无遮挡的直线路径，观察机器人运动。示例不发布占据地图。

## 启动参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `world_name` | `flat_small_scale_maze.world` | 仿真场景。 |
| `params_file` | `config/hello_finenav.yaml` | 导航应用配置文件。 |
| `use_rviz` | `true` | 是否启动 RViz。 |

关闭 RViz：

```bash
ros2 launch finenav_example hello_finenav.launch.py use_rviz:=false
```

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
