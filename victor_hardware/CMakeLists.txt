cmake_minimum_required(VERSION 3.10.0)
project(victor_hardware)

set(CMAKE_CXX_STANDARD 20)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        ament_cmake_python
        ament_cmake_gtest
        pluginlib
        hardware_interface
        arm_utilities
        lcm
        victor_hardware_interfaces
        rclcpp
        rclcpp_lifecycle
        rclpy
)

foreach (THIS_PACKAGE_INCLUDE_DEPEND ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${THIS_PACKAGE_INCLUDE_DEPEND} REQUIRED)
endforeach ()

# make all warnings into errors
add_compile_options(-Werror)

include_directories(include lcm_types SYSTEM)

# C++ Library (Plugin) for use with ros2_control
add_library(victor_hardware
        SHARED
        src/victor_hardware_interface.cpp
)
target_include_directories(
        victor_hardware
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(victor_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Node for ROS <--> LCM bridge
add_executable(iiwa_lcm_bridge_node
        src/iiwa_lcm_bridge_node.cpp
        src/iiwa_lcm_bridge_utils.cpp
        src/iiwa_lcm_bridge.cpp
        src/robotiq_3f_lcm_bridge.cpp
        include/victor_hardware/victor_utils.hpp
)
target_include_directories(
        iiwa_lcm_bridge_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(iiwa_lcm_bridge_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Testing
ament_add_gtest(test_victor_utils tests/test_victor_utils.cpp)
ament_target_dependencies(test_victor_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
ament_python_install_package(${PROJECT_NAME})

install(
        TARGETS iiwa_lcm_bridge_node victor_hardware
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include DESTINATION .)
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)
# Install Python nodes
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.py"
)

pluginlib_export_plugin_description_file(hardware_interface victor_hardware.xml)

# Export
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(victor_hardware)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)

ament_package()