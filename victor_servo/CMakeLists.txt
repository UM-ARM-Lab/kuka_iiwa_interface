cmake_minimum_required(VERSION 3.8)
project(victor_servo)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
    moveit_servo)

foreach(dep ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# Build
add_executable(victor_servo_node src/victor_servo_node.cpp)
target_include_directories(victor_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(victor_servo_node ${THIS_PACKAGE_DEPENDS})

# Export

# Install
install(TARGETS victor_servo_node DESTINATION lib/${PROJECT_NAME})

ament_package()
