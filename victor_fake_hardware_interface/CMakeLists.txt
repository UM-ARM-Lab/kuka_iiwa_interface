cmake_minimum_required(VERSION 2.8.3)
project(victor_fake_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  victor_hardware_interface_msgs
)

catkin_python_setup()

