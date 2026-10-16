generate_parameter_library(dummy_world_view_params dummy_world_view.yaml)

add_library(dummy_world_view SHARED src/dummy_world_view.cpp)
target_include_directories(dummy_world_view PUBLIC include)
target_link_libraries(dummy_world_view
    dummy_world_view_params
    dummy_map
    dummy_planner)
ament_target_dependencies(dummy_world_view finenav_core)

install(TARGETS dummy_world_view ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Publish dummy_world_view as a discoverable FineNav algorithm plugin.
finenav_register_algo_plugin(dummy_world_view)
