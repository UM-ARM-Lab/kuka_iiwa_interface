cmake_minimum_required(VERSION 3.8)
project(robotiq_3f_interfaces)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(robotiq_3f_interfaces
  "msg/ObjectDetectionStatus.msg"
  "msg/Status.msg"
  "msg/GraspingMode.msg"
  "msg/SimpleControlCommand.msg"
  "msg/IndependentControlCommand.msg"
  "srv/ChangeGraspingMode.srv"
  DEPENDENCIES
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
