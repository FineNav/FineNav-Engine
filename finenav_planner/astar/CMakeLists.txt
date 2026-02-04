cmake_minimum_required(VERSION 3.22)
project(finenav_planner_astar)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

generate_parameter_library(
        astar_params
        astar.yaml
)

ament_auto_add_executable(test_executable src/test.cpp)
target_link_libraries(test_executable astar_params)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package()