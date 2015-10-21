# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required( VERSION 2.8.3 )

project( lib_vision )

if ( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++1y " )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++1y -O3" )

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package( catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    lib_atlas
)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES sonia_vision
    CATKIN_DEPENDS roscpp cv_bridge lib_atlas
)

find_package( OpenCV REQUIRED)

#===============================================================================
# O T H E R   P A C K A G E S   D E P E N D E N C I E S

#===============================================================================
# I N C L U D E   F I L E S

set( lib_vision_SRC_DIR "src" )

# Get all the files of the projects.
file( GLOB_RECURSE lib_vision_HDR_FILES "${lib_vision_SRC_DIR}/*.h" )
file( GLOB_RECURSE lib_vision_SRC_FILES "${lib_vision_SRC_DIR}/*.cpp" )

# Include all the directories where source can be found.
include_directories(
    ${lib_vision_SRC_DIR}
    ${catkin_INCLUDE_DIRS}
    ${lib_atlas_INCLUDE_DIRS}
)

#===============================================================================
# C R E A T E   E X E C U T A B L E

set( TARGET_NAME sonia_vision )

SET( LIBS_TO_LINK
  ${catkin_LIBRARIES}
  ${lib_atlas_LIBRARIES}
  ${OpenCV_LIBS}
)

add_library( ${TARGET_NAME} ${lib_vision_SRC_FILES} )
target_link_libraries( ${TARGET_NAME} ${LIBS_TO_LINK})

set_target_properties( ${TARGET_NAME}
    PROPERTIES
    LINKER_LANGUAGE CXX
)
SET( LIBS_TO_LINK ${LIBS_TO_LINK} ${TARGET_NAME})
#==========================================================================
# C R E A T I N G   U N I T   T E S T S

catkin_add_gtest( utest_object_ranking_data test/object_ranking_data_test.cc )
target_link_libraries( utest_object_ranking_data ${LIBS_TO_LINK})

catkin_add_gtest( utest_object_tracking_data test/object_tracking_data_test.cc )
target_link_libraries( utest_object_tracking_data  ${LIBS_TO_LINK})

catkin_add_gtest( utest_object_ranker test/object_ranker_test.cc )
target_link_libraries(  utest_object_ranker ${LIBS_TO_LINK})

catkin_add_gtest( utest_object_full_data test/object_full_data_test.cc )
target_link_libraries(  utest_object_full_data ${LIBS_TO_LINK})

catkin_add_gtest( utest_object_frame_memory test/object_frame_memory_test.cc )
target_link_libraries( utest_object_frame_memory ${LIBS_TO_LINK})

catkin_add_gtest( utest_object_basic_data test/object_basic_data_test.cc )
target_link_libraries(  utest_object_basic_data ${LIBS_TO_LINK})

catkin_add_gtest( utest_feature_factory test/feature_factory_test.cc )
target_link_libraries( utest_feature_factory ${LIBS_TO_LINK})

catkin_add_gtest( utest_performance_evaluator test/performace_evaluator_test.cc )
target_link_libraries( utest_performance_evaluator ${LIBS_TO_LINK})

catkin_add_gtest( utest_contour test/contour_test.cc )
target_link_libraries( utest_contour ${LIBS_TO_LINK})


#==========================================================================
# I N S T A L L I N G   T A R G E T S

install( TARGETS ${TARGET_NAME}
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
