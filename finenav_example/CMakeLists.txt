cmake_minimum_required(VERSION 3.22)
project(finenav_example)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(finenav_engine_cpp REQUIRED)
find_package(finenav_cmake REQUIRED)

add_subdirectory(create_custom_map)
add_subdirectory(create_custom_planner)
add_subdirectory(create_custom_world_view)
add_subdirectory(getting_started)

ament_package()
