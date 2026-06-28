find_package(benchmark REQUIRED)

# ── MPPI source files compiled directly into benchmark ──
# This isolates benchmark build flags from production shared libraries.
set(MPPI_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critic_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/noise_generator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/cost_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/goal_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/goal_angle_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/path_align_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/path_follow_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/path_angle_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/prefer_forward_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/twirling_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/constraint_critic.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/critics/velocity_deadband_critic.cpp
  # OBB collision model (same as production)
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../finenav_collision_model/src/obb_collision_model.cpp
)

add_executable(finenav_benchmark
  finenav_benchmark.cpp
  bench_map_view.hpp
  ${MPPI_SRCS}
)

ament_target_dependencies(finenav_benchmark ${dependencies_pkgs})
target_link_libraries(finenav_benchmark benchmark finenav_mppi_controller_params)
target_include_directories(finenav_benchmark PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${CMAKE_BINARY_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../finenav_collision_model/include
)

# ── Debug / timing flags isolated to benchmark only ──
# Enable with: colcon build --cmake-args -DBENCH_ASAN=ON -DBENCH_TIMING=ON
option(BENCH_ASAN    "Build benchmark with AddressSanitizer"  OFF)
option(BENCH_TIMING  "Build benchmark with MPPI_TIMING"       OFF)

if(BENCH_TIMING)
  target_compile_definitions(finenav_benchmark PRIVATE MPPI_TIMING)
endif()

if(BENCH_ASAN)
  target_compile_options(finenav_benchmark    PRIVATE -fsanitize=address -fno-omit-frame-pointer -g -O1)
  target_link_options(finenav_benchmark       PRIVATE -fsanitize=address)
endif()
