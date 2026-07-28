cmake_minimum_required(VERSION 3.22)
project(finenav_route_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

# ── Dependencies — explicit find_package, following nav2_mppi_controller ────
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_util REQUIRED)
find_package(finenav_interfaces REQUIRED)
find_package(finenav_map_occ_grid_2d REQUIRED)
find_package(finenav_collision_model REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ── parameter libraries (generated from YAML) ──────────────────────────────
generate_parameter_library(astar_params  astar.yaml)
generate_parameter_library(bfs_params    bfs.yaml)
generate_parameter_library(route_params  route.yaml)
generate_parameter_library(dstarlite_params dstarlite.yaml)
generate_parameter_library(detour_params detour.yaml)

# ── planner library ─────────────────────────────────────────────────────────
add_library(finenav_route_planner_lib SHARED
    # A* engine
    src/astar/astar_algorithm.cpp
    # D* Lite engine
    src/dstarlite/dstarlite_algorithm.cpp
    # Planner wrappers (FineNav PlannerConcept)
    src/astar_path_search.cpp
    src/bfs_path_search.cpp          # BFS = AStarAlgorithm(Heuristic::Zero)
    src/dstarlite_path_search.cpp
    src/route_path_search.cpp
    # Local detour planner (FineNav PlannerConcept)
    src/detour_planner.cpp
    # Route internals — graph loading (GeoJSON) + Dijkstra search
    src/route/route_graph.cpp
    src/route/route_search.cpp
)
target_compile_options(finenav_route_planner_lib PUBLIC -O3)
target_include_directories(finenav_route_planner_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(finenav_route_planner_lib PUBLIC
    astar_params
    bfs_params
    route_params
    dstarlite_params
    detour_params
    finenav_core::finenav_core
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

# ── install ─────────────────────────────────────────────────────────────────
install(TARGETS finenav_route_planner_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS astar_params bfs_params route_params dstarlite_params detour_params
  EXPORT export_${PROJECT_NAME}_params
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ── tests ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
    add_subdirectory(benchmark)
endif ()

# ── standalone eval (always built, no ROS deps) ─────────────────────────────
add_subdirectory(eval)

# ── exports ─────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_targets(export_${PROJECT_NAME}_params HAS_LIBRARY_TARGET)
ament_export_dependencies(
  finenav_core
  Eigen3
)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
