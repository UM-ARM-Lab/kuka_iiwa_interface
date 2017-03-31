cmake_minimum_required(VERSION 2.8.3)
project(iiwa_robot_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arc_utilities
  geometry_msgs
  trajectory_msgs
  control_msgs
  roscpp
  sensor_msgs
  std_msgs
  kdl_parser
  message_generation
  linux_fri_client
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES PositionCommand.msg VelocityCommand.msg TorqueCommand.msg FRIState.msg FRICommand.msg)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES SetInjectedError.srv)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include
#  LIBRARIES baxter_robot_controllers
CATKIN_DEPENDS arc_utilities geometry_msgs trajectory_msgs control_msgs roscpp sensor_msgs std_msgs kdl_parser message_runtime linux_fri_client DEPENDS Eigen3)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wconversion")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -O3 -g")

## Declare C++ executables
add_executable(iiwa_position_controller include/${PROJECT_NAME}/iiwa_robot_config.hpp include/${PROJECT_NAME}/iiwa_robot_position_controller.hpp src/iiwa_position_controller.cpp)
add_dependencies(iiwa_position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iiwa_position_controller ${catkin_LIBRARIES})

add_executable(iiwa_FRI_state_position_torque_shim src/iiwa_FRI_state_position_torque_shim.cpp)
add_dependencies(iiwa_FRI_state_position_torque_shim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iiwa_FRI_state_position_torque_shim ${catkin_LIBRARIES})

add_executable(iiwa_FRI_position_interface src/iiwa_FRI_position_interface.cpp)
add_dependencies(iiwa_FRI_position_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iiwa_FRI_position_interface ${catkin_LIBRARIES})

add_executable(iiwa_joint_state_publisher src/iiwa_joint_state_publisher.cpp)
add_dependencies(iiwa_joint_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iiwa_joint_state_publisher ${catkin_LIBRARIES})

add_executable(iiwa_velocity_torque_controller include/${PROJECT_NAME}/iiwa_robot_config.hpp include/${PROJECT_NAME}/iiwa_robot_velocity_torque_controller.hpp src/iiwa_velocity_torque_controller.cpp)
add_dependencies(iiwa_velocity_torque_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iiwa_velocity_torque_controller ${catkin_LIBRARIES})
