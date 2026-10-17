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
They prepare the workspace for later component selection; the introductory
dummy example uses only the example plugins inside the Engine repository.

```text
finenav_ws/
└── src/
    ├── FineNav-Engine/
    ├── finenav_map/
    └── finenav_planner/
```

## 3. Install dependencies

`python3-rosdep` provides the `rosdep` command. It reads dependencies declared
in `package.xml`, resolves package names for your system, and calls package
managers such as apt to install missing dependencies.
`init` configures the dependency-rule sources once per machine; skip the first
line if already initialized. `update` refreshes the current user's dependency
mapping cache. Run it as a regular user, without `sudo`.

```bash
sudo rosdep init
rosdep update
```

Install the dependencies declared by Engine packages, then add xacro and
PyYAML, which the current simulator scripts use but do not yet declare:

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/FineNav-Engine \
  --ignore-src --rosdistro humble -y
sudo apt install -y ros-humble-xacro python3-yaml libbenchmark-dev
```

| Option | Purpose |
| --- | --- |
| `--from-paths src/FineNav-Engine` | Recursively read Engine manifests, without scanning the independent Map and Planner repositories. |
| `--ignore-src` | Leave packages whose source is in the scanned paths for colcon to build, rather than installing matching system packages. |
| `--rosdistro humble` | Use the ROS 2 Humble dependency mappings. |
| `-y` | Automatically confirm installation. |

`libbenchmark-dev` provides Google Benchmark headers, libraries, and CMake
configuration, and is included following installation feedback. This rosdep
command scans only Engine; it does not install the `benchmark` dependency
declared in the independent Planner repository.

Gazebo bridge and simulation packages are already declared and installed by
rosdep; Humble Desktop includes RViz. When integrating components from the
independent Map and Planner repositories, also resolve the selected components'
dependencies. See their catalogues for integration details.

After installation, check the declared dependencies:

```bash
rosdep check --from-paths src/FineNav-Engine \
  --ignore-src --rosdistro humble
```

The simulator uses Gazebo Sim / Fortress and needs a graphical desktop.
The tutorial selects Gazebo odometry, so Fast-LIO is not needed for this route.
If rosdep reports an unresolved key or an unavailable package, resolve that
dependency before building; a source-only dependency must be cloned in a
compatible version. rosdep does not automatically clone source repositories,
infer undeclared dependencies from CMake, or build the project. A successful
`check` confirms only that declared dependencies are installed; building and
running the example are still required to validate the setup.

## 4. Build the tutorial packages

```bash
cd ~/finenav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to finenav_example finenav_simulator \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 pkg executables finenav_example
```

The output should include `finenav_example hello_finenav`. `--packages-up-to`
builds these two packages and their workspace dependencies.
`-DBUILD_TESTING=OFF` disables test and evaluation targets for the introductory
run. Optional algorithms from the independent Map and Planner repositories
are not included in this build.

Source the environment in **every new terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/finenav_ws/install/setup.bash
```

If a package is not found, check the build output and that you sourced the intended workspace.
Commands here have been checked against the repository files; a clean Ubuntu
build and graphical simulation have not been run as part of this documentation update.

Continue with [1. Run the example](quickstart.md).
