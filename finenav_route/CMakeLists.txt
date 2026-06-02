cmake_minimum_required(VERSION 3.22)
project(finenav_route CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(finenav_planner_bfs REQUIRED)

include_directories(include)

set(library_name finenav_route_core)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nlohmann_json
  finenav_msgs
)

add_library(${library_name} SHARED
  src/route_planner.cpp
  src/route_path_planner.cpp
  src/path_converter.cpp
  src/plugins/graph_file_loaders/geojson_graph_file_loader.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(compute_path_bridge_node
  src/compute_path_bridge_node.cpp
)
ament_target_dependencies(compute_path_bridge_node
  ${dependencies}
  finenav_planner_bfs
)
target_link_libraries(compute_path_bridge_node
  ${library_name}
  ${finenav_planner_bfs_LIBRARIES}
  finenav_msgs::finenav_msgs__rosidl_typesupport_cpp
)
target_include_directories(compute_path_bridge_node PRIVATE
  ${finenav_planner_bfs_INCLUDE_DIRS}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS compute_path_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
