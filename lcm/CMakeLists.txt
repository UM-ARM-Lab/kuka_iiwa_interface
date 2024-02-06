cmake_minimum_required(VERSION 3.10.0)
project(lcm)

find_package(ament_cmake REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(GLib REQUIRED)
set(GLib_LIBRARIES ${GLib_LIBRARY})

include(GNUInstallDirs)


###########
## Build ##
###########
message(AUTHOR_WARNING "------- Deprecated definitions disabled for LCM build")

add_definitions(-Wno-deprecated-declarations)

add_library(${PROJECT_NAME} SHARED
        include/${PROJECT_NAME}/dbg.h
        include/${PROJECT_NAME}/lcm.h
        include/${PROJECT_NAME}/lcm_internal.h
        include/${PROJECT_NAME}/lcm_coretypes.h
        include/${PROJECT_NAME}/ringbuffer.h
        include/${PROJECT_NAME}/udpm_util.h
        include/${PROJECT_NAME}/eventlog.h
        include/${PROJECT_NAME}/ioutils.h
        include/${PROJECT_NAME}/lcm-cpp.hpp
        include/${PROJECT_NAME}/lcm-cpp-impl.hpp
        include/${PROJECT_NAME}/lcmtypes/channel_port_map_update_t.h
        include/${PROJECT_NAME}/lcmtypes/channel_to_port_t.h
        src/${PROJECT_NAME}/lcm.c
        src/${PROJECT_NAME}/lcm_memq.c
        src/${PROJECT_NAME}/lcm_file.c
        src/${PROJECT_NAME}/ringbuffer.c
        src/${PROJECT_NAME}/udpm_util.c
        src/${PROJECT_NAME}/eventlog.c
        src/${PROJECT_NAME}/lcm_mpudpm.c
        src/${PROJECT_NAME}/lcm_udpm.c
        src/${PROJECT_NAME}/lcm_udp.c
        src/${PROJECT_NAME}/lcm_tcpq.c
        src/${PROJECT_NAME}/lcmtypes/channel_port_map_update_t.c
        src/${PROJECT_NAME}/lcmtypes/channel_to_port_t.c)
target_link_libraries(${PROJECT_NAME} ${GLib_LIBRARIES})
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${GLib_INCLUDE_DIRS})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)
ament_export_targets(${PROJECT_NAME}_targets)
ament_export_include_directories(
        include
)

ament_package()
