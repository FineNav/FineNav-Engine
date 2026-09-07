# first start
cd ***_ws //切换到你的工作空间
colcon build --packages-select finenav_simulator
source install/setup.bash

# for real_sim
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
source install/setup.bash
export DISPLAY=:0

ros2 launch finenav_simulator finenav_simulator.launch.py \
  use_sim_time:=true \
  world_name:=flat_small_scale_maze.world

# 定位链:仿真器统一使用 Gazebo 真值里程计(/odom),无模式切换。

# 以下是可选的world，将参数替换为world_name，添加额外的world记得调整导航车出生点
https://github.com/ZJU-FAST-Lab/uneven_planner //不平坦地形
map_desert.world
map_hill.world
map_vocano.world

https://ieeexplore.ieee.org/abstract/document/9636611 //小迷宫地图
flat_small_scale_maze.world

# 遥控操作指南（Gazebo 原生 Teleop 面板）
点击 Play 运行仿真后，在右侧 Teleop 面板中有三个 Tab：
  Button   — 点击方向按钮控制移动
  Keyboard — 键盘 WASD 控制移动（最常用）
  Sliders  — 拖动滑块调节线速度 / 角速度
