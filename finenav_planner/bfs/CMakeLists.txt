cmake_minimum_required(VERSION 3.22)
project(finenav_planner_bfs)

set(CMAKE_CXX_STANDARD 20)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(include)
ament_auto_find_build_dependencies()

generate_parameter_library(
    bfs_params
    bfs.yaml
)

ament_auto_add_library(bfs_planner_lib
    src/bfs_algorithm.cpp
    src/bfs_path_search.cpp
    src/occ_grid_map_view.cpp
)
target_link_libraries(bfs_planner_lib bfs_params ${OpenCV_LIBS})

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(bfs_test
        test/bfs_test.cpp
        src/bfs_algorithm.cpp
        src/bfs_path_search.cpp
    )
    ament_target_dependencies(bfs_test
        finenav_core
        finenav_util
        finenav_msgs
    )
    target_include_directories(bfs_test PRIVATE test)
    target_link_libraries(bfs_test bfs_params)
endif ()

ament_auto_package()
