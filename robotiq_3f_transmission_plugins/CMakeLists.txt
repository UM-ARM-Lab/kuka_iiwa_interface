cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_transmission_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        transmission_interface
        pluginlib
        rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

add_library(individual_control_transmission SHARED
        src/individual_control_transmission.cpp
        src/individual_control_transmission_loader.cpp
)
target_include_directories(individual_control_transmission PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(individual_control_transmission PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Tell downstream packages where to find our headers.
ament_export_include_directories(
        include
)

###############################################################################
# INSTALL

install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS individual_control_transmission
        EXPORT export_individual_control_transmission # Must match the first argument to ament_export_targets() below
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)


###############################################################################
# PLUGINS

pluginlib_export_plugin_description_file(transmission_interface transmission_plugins.xml)

ament_export_targets(export_individual_control_transmission HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
