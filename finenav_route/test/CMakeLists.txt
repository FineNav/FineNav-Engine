ament_add_gtest(test_utils_and_types
  test_utils_and_types.cpp
)
target_link_libraries(test_utils_and_types
  ${library_name}
)

ament_add_gtest(test_geojson_graph_file_loader
  test_geojson_graph_file_loader.cpp
)
target_link_libraries(test_geojson_graph_file_loader
  ${library_name}
)

ament_add_gtest(test_spatial_tree
  test_spatial_tree.cpp
)
target_link_libraries(test_spatial_tree
  ${library_name}
)

ament_add_gtest(test_route_planner
  test_route_planner.cpp
)
target_link_libraries(test_route_planner
  ${library_name}
)

ament_add_gtest(test_path_converter
  test_path_converter.cpp
)
target_link_libraries(test_path_converter
  ${library_name}
)
