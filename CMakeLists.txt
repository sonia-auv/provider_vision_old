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

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3 -s -Wno-unknown-pragmas")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

set(provider_vision_SRC_DIR "src")

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        image_transport
        cv_bridge
        roslaunch
        lib_atlas
        sonia_msgs
        dynamic_reconfigure
        )

generate_dynamic_reconfigure_options(
        cfg/Camera_Parameters_.cfg
)

find_package(DC1394 REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
        INCLUDE_DIRS ${provider_vision_SRC_DIR}
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        roslaunch
        lib_atlas
        sonia_msgs
        dynamic_reconfigure
)

roslaunch_add_file_check(launch)

#===============================================================================
# I N C L U D E   F I L E S

file(GLOB_RECURSE provider_vision_FILES
        "${provider_vision_SRC_DIR}/*.cc"
        "${provider_vision_SRC_DIR}/*.h")

list(REMOVE_ITEM provider_vision_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/${provider_vision_SRC_DIR}/${PROJECT_NAME}/main.cc)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${provider_vision_SRC_DIR}
        ${lib_atlas_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

include_directories(
        $ENV{GENICAM_ROOT_V3_0}/library/CPP/include/
        $ENV{GIGEV_DIR}/include/
)

#===============================================================================
# C R E A T E   E X E C U T A B L E

add_executable(${PROJECT_NAME}_node ${provider_vision_SRC_DIR}/${PROJECT_NAME}/main.cc ${provider_vision_FILES} )
target_link_libraries(provider_vision_node
        ${catkin_LIBRARIES}
        ${lib_atlas_LIBRARIES}
        ${OpenCV_LIBRARIES}
        yaml-cpp
        dc1394
        $ENV{GIGEV_DIR}/lib/libGevApi.so.2.0
        $ENV{GENICAM_ROOT}/bin/Linux64_x64/libNodeMapData_gcc421_v3_0.so
        $ENV{GENICAM_ROOT}/bin/Linux64_x64/libXmlParser_gcc421_v3_0.so
        $ENV{GENICAM_ROOT}/bin/Linux64_x64/libGenApi_gcc421_v3_0.so
        $ENV{GENICAM_ROOT}/bin/Linux64_x64/libGCBase_gcc421_v3_0.so
        )
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_generate_messages_cpp
        ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

#============================================================================
# U N I T   T E S T S

# Uncomment this line to compile UT.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
