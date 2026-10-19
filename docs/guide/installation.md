# Installation

Prepare the workspace, install dependencies, and build FineNav-Engine.

## 1. Prepare Ubuntu and ROS 2

Use **Ubuntu 22.04 and ROS 2 Humble**.
Install Humble Desktop using the
[official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Run the commands below in an Ubuntu terminal.

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
```

## 2. Clone the repository

```bash
mkdir -p ~/finenav_ws/src
cd ~/finenav_ws
git clone --branch main https://github.com/FineNav/FineNav-Engine.git src/FineNav-Engine
```

## 3. Install dependencies

```bash
sudo rosdep init  # Skip if already initialized.
rosdep update

cd ~/finenav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/FineNav-Engine --ignore-src --rosdistro humble -y
sudo apt install -y ros-humble-xacro python3-yaml libbenchmark-dev
```

## 4. Build

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Launch the example

Start the simulator, navigation application, and RViz together:

```bash
ros2 launch finenav_example hello_finenav.launch.py
```

See [1. Run the example](quickstart.md) for simulator configuration and goal input.
