# Installation

Download the source and build the packages used throughout Getting Started.

## 1. Prepare Ubuntu and ROS 2

Use **Ubuntu 22.04 and ROS 2 Humble**, the prerequisites listed by the example.
Install ROS 2 Humble Desktop using the
[official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Run the commands below in an Ubuntu terminal.

```bash
sudo apt update
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
```

## 2. Clone the repositories

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone --branch dev https://github.com/FineNav/FineNav-Engine.git src/FineNav-Engine
git clone https://github.com/FineNav/finenav_map.git src/finenav_map
git clone https://github.com/FineNav/finenav_planner.git src/finenav_planner
```

Use an account with repository access if GitHub asks you to authenticate.
The map and planner repositories use their default branches.

```text
finenav_ws/
└── src/
    ├── FineNav-Engine/
    ├── finenav_map/
    └── finenav_planner/
```

## 3. Install dependencies

`python3-rosdep` installs the `rosdep` command. It reads ROS package manifests
and resolves declared system dependencies; on Ubuntu, most are installed with apt.
Initialize it once per machine; skip the first line if already initialized.

```bash
sudo rosdep init
rosdep update
```

For the Engine example, resolve the dependencies declared by Engine packages:

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/FineNav-Engine --ignore-src --rosdistro humble -y
sudo apt install -y ros-humble-ros-gz ros-humble-rviz2 \
  ros-humble-xacro python3-yaml
```

The simulator uses Gazebo Sim / Fortress and needs a graphical desktop.
The tutorial selects Gazebo odometry, so Fast-LIO is not needed for this route.
If rosdep reports an unresolved key or an unavailable package, resolve that
dependency before building; a source-only dependency must be cloned in a
compatible version. rosdep does not generate or download missing source code.

## 4. Build the tutorial packages

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to finenav_example finenav_simulator
source install/setup.bash
ros2 pkg executables finenav_example
```

The output should include `finenav_example hello_finenav`. `--packages-up-to`
builds these two packages and their workspace dependencies. The independent
Map and Planner repositories are present in `src`, but their optional algorithms
are not required by the dummy example. See their catalogues before integrating them.

Source the environment in **every new terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
```

If a package is not found, check the build output and that you sourced the intended workspace.
Commands here have been checked against the repository files; a clean Ubuntu
build and graphical simulation have not been run as part of this documentation update.

Continue with [1. Run the example](quickstart.md).
