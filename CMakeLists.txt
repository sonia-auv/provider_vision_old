# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)

project(provider_vision)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

set(provider_vision_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    sonia_msgs
    roscpp
    message_generation
    message_runtime
    image_transport
    cv_bridge
    roslaunch
    lib_atlas
    lib_vision
    )

find_package(DC1394 REQUIRED)
find_package(OpenCV REQUIRED)

#===============================================================================
# C R E A T E   C A T K I N   P A C K A G E

catkin_package(
    INCLUDE_DIRS ${provider_vision_SRC_DIR}
    LIBRARIES
    CATKIN_DEPENDS
    sonia_msgs
    roscpp
    image_transport
    cv_bridge
    roslaunch
    lib_vision
)

roslaunch_add_file_check(launch)

#===============================================================================
# I N C L U D E   F I L E S

# Get all the files of the projects.
file(GLOB_RECURSE provider_vision_FILES
    "${provider_vision_SRC_DIR}/provider_vision/media/*.h"
    "${provider_vision_SRC_DIR}/provider_vision/media/*.cc"
    "${provider_vision_SRC_DIR}/provider_vision/server/*.h"
    "${provider_vision_SRC_DIR}/provider_vision/server/*.cc"
    "${provider_vision_SRC_DIR}/provider_vision/proc/*.h"
    "${provider_vision_SRC_DIR}/provider_vision/proc/*.cc"
    "${provider_vision_SRC_DIR}/provider_vision/utils/*.h"
    "${provider_vision_SRC_DIR}/provider_vision/utils/*.cc"
    "${provider_vision_SRC_DIR}/provider_vision/utils/*.cpp" # For pugixml
    )

# Include all the directories where source can be found.
include_directories(
    ${provider_vision_SRC_DIR}
    ${lib_atlas_INCLUDE_DIRS}
    ${lib_vision_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

#===============================================================================
# C R E A T E   E X E C U T A B L E

# Create a CXX file from the given sources
add_executable(provider_vision_node ${provider_vision_SRC_DIR}/provider_vision/main.cc
    ${provider_vision_FILES}
    )
# Link all the used libraries to the previsouly created executable.
target_link_libraries(provider_vision_node
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif ()
