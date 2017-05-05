cmake_minimum_required(VERSION 2.8.3)
project(linux_fri_client)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(cmake_modules REQUIRED)

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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp DEPENDS)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "-std=c99 ${CMAKE_C_FLAGS} -Wall -Wextra -Wconversion -O3 -g")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wall -Wextra -Wconversion -O3 -g")

## Declare C++ library
set(FRI_CLIENT_SOURCES
    src/${PROJECT_NAME}/friClientApplication.cpp
    include/${PROJECT_NAME}/friClientApplication.h
    include/${PROJECT_NAME}/friClientData.h
    include/${PROJECT_NAME}/friClientIf.h
    src/${PROJECT_NAME}/friCommandMessageEncoder.cpp
    include/${PROJECT_NAME}/friCommandMessageEncoder.h
    include/${PROJECT_NAME}/friConnectionIf.h
    src/${PROJECT_NAME}/friLBRClient.cpp
    include/${PROJECT_NAME}/friLBRClient.h
    src/${PROJECT_NAME}/friLBRCommand.cpp
    include/${PROJECT_NAME}/friLBRCommand.h
    src/${PROJECT_NAME}/friLBRState.cpp
    include/${PROJECT_NAME}/friLBRState.h
    src/${PROJECT_NAME}/FRIMessages.pb.c
    include/${PROJECT_NAME}/FRIMessages.pb.h
    src/${PROJECT_NAME}/friMonitoringMessageDecoder.cpp
    include/${PROJECT_NAME}/friMonitoringMessageDecoder.h
    src/${PROJECT_NAME}/friUdpConnection.cpp
    include/${PROJECT_NAME}/friUdpConnection.h
    src/${PROJECT_NAME}/pb_decode.c
    include/${PROJECT_NAME}/pb_decode.h
    src/${PROJECT_NAME}/pb_encode.c
    include/${PROJECT_NAME}/pb_encode.h
    src/${PROJECT_NAME}/pb_frimessages_callbacks.c
    include/${PROJECT_NAME}/pb_frimessages_callbacks.h
    include/${PROJECT_NAME}/pb.h
)

add_library(${PROJECT_NAME} ${FRI_CLIENT_SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
