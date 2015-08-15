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

IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo\
       MinSizeRel."
      FORCE )
ENDIF( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++1y " )
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
  cv_bridge
  image_transport
  rqt_gui_cpp
  roslaunch
)
find_package( catkin REQUIRED lib_atlas )
find_package( catkin REQUIRED lib_vision )

roslaunch_add_file_check( launch )

add_message_files(
  DIRECTORY msg
  FILES
  image_feed.msg
  filterchain_return_message.msg
)

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

catkin_package( CATKIN_DEPENDS
    std_msgs
    roscpp
    rospy
    message_runtime
    # Sonia packages
    lib_atlas
    lib_vision
)

#===============================================================================
# O T H E R   P A C K A G E S   D E P E N D E N C I E S

find_package( DC1394 REQUIRED )

find_package( OpenCV REQUIRED )

#===============================================================================
# I N C L U D E   F I L E S

# Defining the sources and headers directories.
set( vision_server_SRC_DIR "src" )

# Get all the files of the projects.
file( GLOB_RECURSE vision_server_FILES
  "${vision_server_SRC_DIR}/*.h"
  "${vision_server_SRC_DIR}/*.cpp"
)

# Include all the directories where source can be found.
include_directories(
  "/usr/local/include/vitals"
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${vision_server_SRC_DIR}
  ${lib_atlas_INCLUDE_DIRS}
  ${lib_vision_INCLUDE_DIRS}
)

#===============================================================================
# C R E A T E   E X E C U T A B L E

# Create a CXX file from the given sources
add_executable(vision_server_node
    ${vision_server_FILES}
)

add_dependencies( vision_server_node ${catkin_EXPORTED_TARGETS} )

# Link all the used libraries to the previsouly created executable.
target_link_libraries( vision_server_node
    vitals
    dc1394
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${image_transport_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
)

# As we use this folder on our project, we want it to be created before runtime.
add_custom_command( TARGET vision_server_node
    PRE_BUILD COMMAND mkdir -p "$ENV{ROS_SONIA_WS}/log"
)
