cmake_minimum_required(VERSION 3.22)
project(finenav_planner_neupan)
set(CMAKE_CXX_STANDARD 20)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

ament_auto_find_build_dependencies()

# ── parameter library (generated from neupan.yaml) ─────────────────────────
generate_parameter_library(
    neupan_params
    neupan.yaml
)

# ── planner library ─────────────────────────────────────────────────────────
ament_auto_add_library(neupan_planner_lib
    src/neupan_plan.cpp
)

target_link_libraries(neupan_planner_lib
    neupan_params
    pybind11::embed
    Python3::Python
)

ament_auto_package()
