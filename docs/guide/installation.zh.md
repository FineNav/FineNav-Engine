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
git clone --branch main https://github.com/FineNav/FineNav-Engine.git src/FineNav-Engine
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
git clone https://github.com/FineNav/finenav_planner.git src/finenav_planner
```

如 GitHub 要求登录，请使用拥有仓库访问权限的账号。
Engine 使用 `main` 分支，地图与规划仓库使用各自默认分支。
Map、Planner 仓库为后续选择组件准备；入门 Dummy 示例仅使用 Engine 仓库内的示例插件。

```text
finenav_ws/
└── src/
    ├── FineNav-Engine/
    ├── finenav_map/
    └── finenav_planner/
```

## 3. 安装依赖

上面的 `python3-rosdep` 软件包提供 `rosdep` 命令。它读取 `package.xml` 中的依赖声明，
查询当前系统对应的软件包，再调用 apt 等包管理器安装缺少的依赖。
`init` 配置依赖规则来源，每台机器只需执行一次；已经初始化过时跳过第一行。
`update` 更新当前用户的依赖映射缓存，使用普通用户执行，不加 `sudo`。

```bash
sudo rosdep init
rosdep update
```

安装 Engine 各包声明的依赖，并补充当前仿真脚本所需、但尚未在清单中声明的 xacro 和 PyYAML：

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/FineNav-Engine \
  --ignore-src --rosdistro humble -y
sudo apt install -y ros-humble-xacro python3-yaml libbenchmark-dev
```

| 参数 | 作用 |
| --- | --- |
| `--from-paths src/FineNav-Engine` | 递归读取 Engine 内的 `package.xml`，不扫描独立 Map、Planner 仓库。 |
| `--ignore-src` | 扫描范围内已有源码的包交给后续 colcon 编译，不尝试安装对应系统包。 |
| `--rosdistro humble` | 使用 ROS 2 Humble 的依赖映射。 |
| `-y` | 自动确认安装。 |

`libbenchmark-dev` 提供 Google Benchmark 的头文件、库和 CMake 配置，按安装实测反馈补装。
当前 rosdep 命令只扫描 Engine；独立 Planner 仓库中的 `benchmark` 依赖不会由此命令安装。

Gazebo 桥接与仿真包已在清单中声明，由 rosdep 安装；Humble Desktop 已包含 RViz。
接入独立 Map、Planner 仓库中的组件时，还需要处理所选组件的依赖，详见对应的组件目录。

安装完成后检查已声明的依赖：

```bash
rosdep check --from-paths src/FineNav-Engine \
  --ignore-src --rosdistro humble
```

仿真使用 Gazebo Sim / Fortress，需要图形桌面。本教程选择 Gazebo 里程计，因此该流程无需安装 Fast-LIO。
如果 rosdep 报依赖键无法解析、或软件包不可用，先解决该依赖再编译；只能从源码获取的依赖，
需要另行克隆兼容版本。rosdep 不会自动克隆源码、读取 CMake 来推断遗漏依赖或编译项目。
`check` 通过只表示已声明依赖安装齐全，仍需通过编译和运行验证。

## 4. 编译示例

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to finenav_example finenav_simulator \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 pkg executables finenav_example
```

输出应包含 `finenav_example hello_finenav`。`--packages-up-to` 会编译这两个包及其工作空间依赖。
`-DBUILD_TESTING=OFF` 关闭测试和评估目标，适合首次运行入门示例。
独立 Map、Planner 仓库中的可选算法不会因此被一并编译。

**每个新终端**都需要加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
```

如果提示找不到软件包，先检查编译结果，以及加载的是否是当前工作空间。
本页命令已对照仓库文件核对；此次文档更新尚未执行干净 Ubuntu 环境下的编译和图形仿真。

下一步：[1. 运行示例](quickstart.md)。
