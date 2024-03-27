cmake_minimum_required(VERSION 3.10)
project(victor_hardware_interfaces)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(victor_hardware_interfaces
  "msg/CartesianValueQuantity.msg"
  "msg/GraspStatus.msg"
  "msg/JointValueQuantity.msg"
  "msg/MotionStatus.msg"
  "msg/Robotiq3FingerActuatorCommand.msg"
  "msg/Robotiq3FingerActuatorStatus.msg"
  "msg/Robotiq3FingerCommand.msg"
  "msg/Robotiq3FingerObjectStatus.msg"
  "msg/Robotiq3FingerStatus.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
