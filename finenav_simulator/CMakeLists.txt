cmake_minimum_required(VERSION 3.22)
project(finenav_simulator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_executable(point_cloud_converter_node
        src/point_cloud_converter.cpp
)

ament_auto_add_executable(simulator_helper_node #TODO: executable
        src/simulator_helper_node.cpp
        src/odom_path_publisher.cpp
        src/ground_truth_map_localizer.cpp
        src/simulator_resetter.cpp
)

target_link_libraries(simulator_helper_node
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
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
