cmake_minimum_required(VERSION 3.5)
project(finenav_mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(
  include
)


set(dependencies_pkgs
  rclcpp
  builtin_interfaces
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  angles
  OpenCV
        finenav_util
        finenav_core
        generate_parameter_library
        finenav_msgs
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()


# ↓ 覆盖 nav2_package() 设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=deprecated-declarations)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ↑ 覆盖 nav2_package() 设置

# ↓ 生成参数
generate_parameter_library(
        finenav_mppi_controller_params
        src/finenav_mppi_controller.yaml
)
# ↑ 生成参数

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

option(ENABLE_COVERAGE "Enable GCC/Clang coverage instrumentation" OFF)
if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

add_library(mppi_controller SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
  src/timing_statistics.cpp
  src/trajectory_visualizer.cpp
)

add_library(mppi_critics SHARED
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/path_rotation_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_definitions(${lib} PUBLIC EIGEN_MAX_ALIGN_BYTES=16)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_link_libraries(${lib} Eigen3::Eigen finenav_mppi_controller_params)
  target_include_directories(${lib} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

target_link_libraries(mppi_controller mppi_critics)
target_compile_options(mppi_critics PRIVATE ${OpenMP_CXX_FLAGS})
set_property(TARGET mppi_critics APPEND_STRING PROPERTY LINK_FLAGS " ${OpenMP_CXX_FLAGS}")

install(
  TARGETS ${libraries} finenav_mppi_controller_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)




install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(timing_statistics_test
    test/timing_statistics_test.cpp
    src/timing_statistics.cpp
  )
  target_include_directories(timing_statistics_test PRIVATE include)

  ament_add_gtest(framework_api_contract_test
    test/framework_api_contract_test.cpp
  )
  ament_target_dependencies(framework_api_contract_test ${dependencies_pkgs})
  target_include_directories(framework_api_contract_test PRIVATE include)
  target_link_libraries(
    framework_api_contract_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )

  ament_add_gtest(path_rotation_critic_test
    test/path_rotation_critic_test.cpp
  )
  ament_target_dependencies(path_rotation_critic_test ${dependencies_pkgs})
  target_include_directories(path_rotation_critic_test PRIVATE include)
  target_link_libraries(
    path_rotation_critic_test
    mppi_critics
    finenav_mppi_controller_params
  )
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs} eigen3_cmake_module Eigen3)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
