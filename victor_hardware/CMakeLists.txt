cmake_minimum_required(VERSION 3.10.0)
project(victor_hardware)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to Release for maximum performance.")
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        controller_interface
        controller_manager_msgs
        pluginlib
        hardware_interface
        arm_utilities
        joint_trajectory_controller
        lcm
        victor_hardware_interfaces
        robotiq_3f_transmission_plugins
        ament_index_cpp
        yaml-cpp
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
        # template implementations
        include/victor_hardware/kuka_control_mode_client.hpp
        include/victor_hardware/lcm_listener.hpp
        # Controllers and broadcasters also get added to this library
        src/kuka_cartesian_controller.cpp
        src/kuka_joint_trajectory_controller.cpp
)
target_include_directories(
        victor_hardware
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(victor_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(victor_hardware yaml-cpp)


install(
        TARGETS victor_hardware
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include DESTINATION .)

pluginlib_export_plugin_description_file(hardware_interface victor_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface controllers.xml)


# Export
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_libraries(victor_hardware)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)

ament_package()