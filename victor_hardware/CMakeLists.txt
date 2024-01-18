cmake_minimum_required(VERSION 3.10.0)
project(victor_hardware)

set(CMAKE_CXX_STANDARD 20)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        ament_cmake_python
        ament_cmake_gtest
        arm_utilities
        lcm
        victor_hardware_interfaces
        rclcpp
        rclpy
)

foreach (THIS_PACKAGE_INCLUDE_DEPEND ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${THIS_PACKAGE_INCLUDE_DEPEND} REQUIRED)
endforeach ()

# make all warnings into errors
add_compile_options(-Werror)

include(GNUInstallDirs)

include_directories(include lcm_types SYSTEM)

add_executable(victor_lcm_bridge_node
        src/victor_lcm_bridge_node.cpp
        src/victor_lcm_bridge_utils.cpp
        src/iiwa_lcm_bridge.cpp
        src/robotiq_3f_lcm_bridge.cpp
        include/victor_hardware/victor_utils.hpp
)
target_include_directories(
        victor_lcm_bridge_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(victor_lcm_bridge_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Testing
ament_add_gtest(test_victor_utils tests/test_victor_utils.cpp)
ament_target_dependencies(test_victor_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
ament_python_install_package(${PROJECT_NAME})

install(
        TARGETS victor_lcm_bridge_node
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

# Export
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)

ament_package()