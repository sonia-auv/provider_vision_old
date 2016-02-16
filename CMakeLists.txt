# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(lib_vision)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release
      RelWithDebInfo MinSizeRel."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s")

set(lib_vision_SRC_DIR "src")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    cv_bridge
    sonia_msgs
    lib_atlas
    )

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS ${lib_vision_SRC_DIR}
    LIBRARIES sonia_vision
    CATKIN_DEPENDS roscpp message_runtime std_msgs cv_bridge sonia_msgs lib_atlas
)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE lib_vision_FILES "${lib_vision_SRC_DIR}/*.h" "${lib_vision_SRC_DIR}/*.cc")

# Include all the directories where source can be found.
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${lib_vision_SRC_DIR}
    ${lib_atlas_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

#===============================================================================
# C R E A T E   E X E C U T A B L E

add_library(sonia_vision ${lib_vision_FILES})
target_link_libraries(sonia_vision ${catkin_LIBRARIES} ${lib_atlas_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(sonia_vision sonia_msgs_generate_messages_cpp)

install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(
    DIRECTORY ${lib_vision_SRC_DIR}/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".git" EXCLUDE
)

#==========================================================================
# U N I T   T E S T S

if (CATKIN_ENABLE_TESTING AND ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif ()
