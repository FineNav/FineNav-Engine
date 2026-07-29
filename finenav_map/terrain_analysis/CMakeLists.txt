cmake_minimum_required(VERSION 3.22)
project(finenav_map_terrain_analysis)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

# ── Generate parameter struct from YAML ──────────────────────────────
generate_parameter_library(
    terrain_analysis_params
    param/terrain_analysis.yaml
)

# ── Library ──────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
    src/terrain_analyzer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/terrain_analysis_params_parameters>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    terrain_analysis_params
    ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    finenav_core
    finenav_map_grid_map
    finenav_map_temporal_voxel_map
    finenav_map_occupancy_grid_2d
    finenav_visualizer
    rclcpp
)

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} terrain_analysis_params
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    finenav_core finenav_map_grid_map finenav_map_temporal_voxel_map
    finenav_map_occupancy_grid_2d finenav_visualizer rclcpp Eigen3 OpenCV
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
