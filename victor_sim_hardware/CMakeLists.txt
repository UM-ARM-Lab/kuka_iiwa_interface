cmake_minimum_required(VERSION 3.8)
project(victor_sim_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(victor_hardware_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Create the hardware interface library
add_library(
  victor_sim_hardware
  SHARED
  src/victor_sim_hardware_interface.cpp
  src/side.cpp
  src/sim_control_mode_client.cpp
)

target_include_directories(
  victor_sim_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  victor_sim_hardware
  rclcpp
  hardware_interface
  pluginlib
  victor_hardware_interfaces
  geometry_msgs
  controller_manager_msgs
  std_srvs
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface victor_sim_hardware.xml)

# Install the library
install(
  TARGETS victor_sim_hardware
  DESTINATION lib
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install Python module
install(
  DIRECTORY victor_sim_hardware/
  DESTINATION lib/python3.10/site-packages/victor_sim_hardware
  FILES_MATCHING PATTERN "*.py"
)

# Install examples
install(
  DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies for downstream packages
ament_export_include_directories(include)
ament_export_libraries(victor_sim_hardware)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  victor_hardware_interfaces
)

ament_package()
