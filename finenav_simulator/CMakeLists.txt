cmake_minimum_required(VERSION 3.22)
project(finenav_simulator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_executable(point_cloud_converter_node
        src/point_cloud_converter.cpp
)

ament_auto_add_executable(simulator_helper_node #TODO: executable
        src/simulator_helper_node.cpp
        src/odom_path_publisher.cpp
        src/ground_truth_map_localizer.cpp
)

# -- Build-time xmacro processing -------------------------------------------
# Process the robot xmacro template into a plain SDF model at build time so it
# can be installed as a standard Gazebo model (discoverable via model:// URI).
find_package(Python3 REQUIRED)

set(GENERATED_MODEL_DIR "${CMAKE_BINARY_DIR}/models/fine_robot")

add_custom_command(
  OUTPUT "${GENERATED_MODEL_DIR}/model.sdf"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED_MODEL_DIR}"
  COMMAND Python3::Interpreter
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_robot_model.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource/robot/model.sdf.xmacro"
    "${GENERATED_MODEL_DIR}/model.sdf"
    "model://fine_robot/meshes/lidar_support_tri_prism.obj"
  DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/resource/robot/model.sdf.xmacro"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_robot_model.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/sdf_launch_utils.py"
  COMMENT "Generating robot model.sdf from xmacro template"
)

add_custom_target(generate_robot_model ALL
  DEPENDS "${GENERATED_MODEL_DIR}/model.sdf"
)

# -- Install the robot as a standard Gazebo model ----------------------------
install(
  FILES "${GENERATED_MODEL_DIR}/model.sdf"
  DESTINATION "share/${PROJECT_NAME}/models/fine_robot"
)
install(
  FILES "models/fine_robot/model.config"
  DESTINATION "share/${PROJECT_NAME}/models/fine_robot"
)
install(
  FILES "resource/robot/meshes/lidar_support_tri_prism.obj"
  DESTINATION "share/${PROJECT_NAME}/models/fine_robot/meshes"
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in"
)

ament_auto_package(
  INSTALL_TO_SHARE
        launch
        resource
        config
        scripts
)
