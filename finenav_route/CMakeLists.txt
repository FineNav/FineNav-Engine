cmake_minimum_required(VERSION 3.22)
project(finenav_route CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nanoflann REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)

generate_parameter_library(
  finenav_route_params
  src/finenav_route.yaml
)

set(library_name finenav_route_core)

set(dependencies
  rclcpp
  geometry_msgs
  nanoflann
  nlohmann_json
  finenav_core
  finenav_msgs
  finenav_util
)

add_library(${library_name} SHARED
  src/graph_route_planner.cpp
  src/route_planner.cpp
  src/node_spatial_tree.cpp
  src/path_converter.cpp
  src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp
)

target_link_libraries(${library_name}
  finenav_route_params
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${library_name})
ament_package()
