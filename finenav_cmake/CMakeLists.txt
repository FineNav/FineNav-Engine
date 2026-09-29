cmake_minimum_required(VERSION 3.22)
project(finenav_cmake)

# finenav_cmake builds no code.  It installs CMake code only:
#
#   cmake/finenav_cmake-extras.cmake          included by finenav_cmakeConfig.cmake
#   cmake/finenav_register_algo_plugin.cmake  the developer-facing macro
#   cmake/finenav_package_hook.cmake          the ament_package() hook
#
# See cmake/finenav_register_algo_plugin.cmake for what a plugin developer writes.

find_package(ament_cmake REQUIRED)

# Installed next to the generated finenav_cmakeConfig.cmake, so that
# ${finenav_cmake_DIR}/finenav_*.cmake resolves for consumers.
install(DIRECTORY cmake/ DESTINATION share/${PROJECT_NAME}/cmake)

# ament_package(CONFIG_EXTRAS ...) installs the extras file into the same
# directory (so it is installed twice — harmless, and exactly what
# rclcpp_components does) and, more importantly, registers it in the generated
# Config.cmake's `_extras` list.  That is what makes find_package(finenav_cmake)
# import ament_register_extension() into the *consumer's* scope and hook the
# manifest emission onto the consumer's own ament_package() call.
ament_package(CONFIG_EXTRAS "cmake/finenav_cmake-extras.cmake")
