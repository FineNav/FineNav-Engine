cmake_minimum_required(VERSION 3.22)
project(finenav_planner_astar)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

# 生成参数库
generate_parameter_library(
        astar_params
        astar.yaml
)

# 原来的 test_executable 可以保留或删除
# ament_auto_add_executable(test_executable src/main.cpp)
# target_link_libraries(test_executable astar_params)

# ===== 新增你的 astar_planner 可执行文件 =====
ament_auto_add_executable(astar_planner
        src/main.cpp
        )

# 如果 astar_params 是你的参数库，可以链接
target_link_libraries(astar_planner astar_params)

ament_auto_package()