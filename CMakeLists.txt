# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required( VERSION 2.8.3 )

project( vision_server )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo\
       MinSizeRel."
      FORCE )
endif( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++1y -Wall " )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++1y -O3" )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${PROJECT_SOURCE_DIR}
)

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package( catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    message_generation
    message_runtime
    image_transport
    cv_bridge
    roslaunch
    lib_atlas
    lib_vision
)

find_package( DC1394 REQUIRED )

#===============================================================================
# G E N E R A T E   M E S S A G E S   &   S E R V I C E S

add_service_files(
    DIRECTORY srv
    FILES
    vision_server_copy_filterchain.srv
    vision_server_execute_cmd.srv
    vision_server_copy_filterchain.srv
    vision_server_get_filterchain_filter.srv
    vision_server_get_filterchain_filter_all_param.srv
    vision_server_get_filterchain_filter_param.srv
    vision_server_get_information_list.srv
    vision_server_get_media_param_list.srv
    vision_server_get_media_param.srv
    vision_server_manage_filterchain_filter.srv
    vision_server_manage_filterchain.srv
    vision_server_save_filterchain.srv
    vision_server_set_filterchain_filter_order.srv
    vision_server_set_filterchain_filter_param.srv
    vision_server_set_media_param.srv
    vision_server_get_filterchain_from_execution.srv
    vision_server_get_media_from_execution.srv
    vision_server_set_filterchain_filter_observer.srv
)

generate_messages( DEPENDENCIES std_msgs )

#===============================================================================
# C R E A T E   C A T K I N   P A C K A G E

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES
    CATKIN_DEPENDS
    std_msgs
    roscpp
    image_transport
    cv_bridge
    roslaunch
    lib_atlas
    lib_vision
)

roslaunch_add_file_check( launch )

#===============================================================================
# I N C L U D E   F I L E S

# Defining the sources and headers directories.
set( vision_server_SRC_DIR "src" )

# Get all the files of the projects.
file( GLOB_RECURSE vision_server_FILES
    "${vision_server_SRC_DIR}/provider_vision/media/*.h"
    "${vision_server_SRC_DIR}/provider_vision/media/*.cpp"
    "${vision_server_SRC_DIR}/provider_vision/server/*.h"
    "${vision_server_SRC_DIR}/provider_vision/server/*.cpp"
    "${vision_server_SRC_DIR}/provider_vision/proc/*.h"
    "${vision_server_SRC_DIR}/provider_vision/proc/*.cpp"
    "${vision_server_SRC_DIR}/provider_vision/utils/*.h"
    "${vision_server_SRC_DIR}/provider_vision/utils/*.cpp"
)

# Include all the directories where source can be found.
include_directories(
    ${vision_server_SRC_DIR}
    ${lib_atlas_INCLUDE_DIRS}
    ${lib_vision_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

#===============================================================================
# C R E A T E   E X E C U T A B L E
# Create a CXX file from the given sources
add_executable( vision_server_node ${vision_server_SRC_DIR}/provider_vision/main.cpp
    ${vision_server_FILES}
)

# Link all the used libraries to the previsouly created executable.
target_link_libraries( vision_server_node
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
)

#===============================================================================
# C R E A T I N G    U N I T    T E S T S
catkin_add_gtest( configuration_handler_test test/configuration_handler_test.cc ${vision_server_FILES})
target_link_libraries( configuration_handler_test
        dc1394
        ${catkin_LIBRARIES}
        ${lib_atlas_LIBRARIES}
        ${lib_vision_LIBRARIES}
)

catkin_add_gtest( media_manager_test test/media_manager_test.cc ${vision_server_FILES})
target_link_libraries( media_manager_test
        dc1394
        ${catkin_LIBRARIES}
        ${lib_atlas_LIBRARIES}
        ${lib_vision_LIBRARIES}
)
