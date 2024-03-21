cmake_minimum_required(VERSION 3.10.0)
project(victor_hardware)

set(CMAKE_CXX_STANDARD 20)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        ament_cmake_gtest
        pluginlib
        hardware_interface
        arm_utilities
        lcm
#        data_tamer_cpp
        victor_hardware_interfaces
        robotiq_3f_transmission_plugins
        rclcpp
        rclcpp_lifecycle
        rclpy
        std_srvs
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
        src/side.cpp
        src/async_executor.cpp
        src/lcm_ros_conversions.cpp
        src/validators.cpp
        include/victor_hardware/lcm_listener.hpp
)
target_include_directories(
        victor_hardware
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(victor_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Testing
ament_add_gtest(test_victor_utils tests/test_victor_utils.cpp)
ament_target_dependencies(test_victor_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
        TARGETS victor_hardware
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include DESTINATION .)

pluginlib_export_plugin_description_file(hardware_interface victor_hardware.xml)

# Export
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(victor_hardware)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)

ament_package()