cmake_minimum_required(VERSION 3.22)
project(finenav_example)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(finenav_engine_cpp REQUIRED)
# Brings in finenav_register_algo_plugin() and hooks the plugin-manifest
# emission onto this package's ament_package() call (see finenav_cmake).
find_package(finenav_cmake REQUIRED)

ament_auto_find_build_dependencies()

add_subdirectory(create_custom_map)
add_subdirectory(create_custom_planner)
add_subdirectory(create_custom_world_view)
add_subdirectory(getting_started)

# ── Algorithm plugin discovery ─────────────────────────────────────────────
# Each plugin library registers itself from the subdirectory that defines it,
# with one finenav_register_algo_plugin(<target>) call next to its add_library().
# There is deliberately nothing to write here: the calls aggregate, and the
# package's single ament-resource-index marker is emitted by finenav_cmake's
# ament_package() hook below — which is also what lets an application author
# write just `loadAlgoPlugins({"finenav_example"})`.

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
