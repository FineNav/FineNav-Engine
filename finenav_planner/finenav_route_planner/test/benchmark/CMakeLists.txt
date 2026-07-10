find_package(benchmark REQUIRED)

add_executable(route_planner_benchmark
    route_planner_benchmark.cpp
    # Core search engines
    ${PROJECT_SOURCE_DIR}/src/astar_algorithm.cpp
    ${PROJECT_SOURCE_DIR}/src/dstarlite_algorithm.cpp
)

target_include_directories(route_planner_benchmark PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..      # for grid_map.hpp
)

target_link_libraries(route_planner_benchmark
    benchmark::benchmark
)
