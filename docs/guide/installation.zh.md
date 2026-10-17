# 如何安装

从下载源码开始，准备 Getting Started 全程使用的工作空间。

## 1. 配置 Ubuntu 和 ROS 2

使用示例说明要求的 **Ubuntu 22.04 + ROS 2 Humble**。
按照 [ROS 2 官方安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
安装 Humble Desktop。以下命令均在 Ubuntu 终端执行。

```bash
sudo apt update
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
```

## 2. 克隆仓库

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone --branch dev https://github.com/FineNav/FineNav-Engine.git src/FineNav-Engine
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
git clone https://github.com/FineNav/finenav_planner.git src/finenav_planner
```

如 GitHub 要求登录，请使用拥有仓库访问权限的账号。
Engine 使用 `dev` 分支，地图与规划仓库使用各自默认分支。

```text
finenav_ws/
└── src/
    ├── FineNav-Engine/
    ├── finenav_map/
    └── finenav_planner/
```

## 3. 安装依赖

上面的 `python3-rosdep` 软件包提供 `rosdep` 命令。它读取 ROS 包的依赖声明，
解析需要的系统软件包；在 Ubuntu 上，大部分依赖由 apt 安装。
每台机器只需初始化一次；已经初始化过时跳过第一行。

```bash
sudo rosdep init
rosdep update
```

首次运行 Engine 示例时，先安装 Engine 各包声明的依赖和仿真所需组件：

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/FineNav-Engine --ignore-src --rosdistro humble -y
sudo apt install -y ros-humble-ros-gz ros-humble-rviz2 \
  ros-humble-xacro python3-yaml
```

仿真使用 Gazebo Sim / Fortress，需要图形桌面。本教程选择 Gazebo 里程计，因此该流程无需安装 Fast-LIO。
如果 rosdep 报依赖键无法解析、或软件包不可用，先解决该依赖再编译；只能从源码获取的依赖，
需要另行克隆兼容版本。rosdep 不会生成或自动下载缺失的源码。

## 4. 编译教程所需的软件包

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to finenav_example finenav_simulator
source install/setup.bash
ros2 pkg executables finenav_example
```

输出应包含 `finenav_example hello_finenav`。`--packages-up-to` 会编译这两个包及其工作空间依赖。
独立 Map、Planner 仓库已放在 `src` 下，但其中的可选算法不是 Dummy 示例的运行依赖。
接入它们前，请查看对应的组件目录。

**每个新终端**都需要加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
```

如果提示找不到软件包，先检查编译结果，以及加载的是否是当前工作空间。
本页命令已对照仓库文件核对；此次文档更新尚未执行干净 Ubuntu 环境下的编译和图形仿真。

下一步：[1. 运行示例，查看效果](quickstart.md)。
