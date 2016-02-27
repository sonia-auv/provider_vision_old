# \file     CMakeLists.txt
# \author   Thibaut Mattio <thibaut.mattio@gmail.com>
# \date     08/05/2015
# \copyright    2015 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the MIT license that can be
# found in the LICENSE file.

catkin_add_gtest(configuration_handler_test server/configuration_handler_test.cc ${provider_vision_FILES})
target_link_libraries(configuration_handler_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${provider_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(media_manager_test server/media_manager_test.cc ${provider_vision_FILES})
target_link_libraries(media_manager_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${provider_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(detection_task_manager_test server/detection_task_manager_test.cc ${provider_vision_FILES})
target_link_libraries(detection_task_manager_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${provider_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(filterchain_manager_test server/filterchain_manager_test.cc ${provider_vision_FILES})
target_link_libraries(filterchain_manager_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${provider_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(utest_vision_server server/vision_server_test.cc  ${provider_vision_FILES})
target_link_libraries(utest_vision_server
dc1394
${catkin_LIBRARIES}
${lib_atlas_LIBRARIES}
${OpenCV_LIBRARIES}
)

catkin_add_gtest( object_ranking_data_test filters/object_ranking_data_test.cc )
target_link_libraries(object_ranking_data_test ${OpenCV_LIBRARIES})

catkin_add_gtest( object_tracking_data_test filters/object_tracking_data_test.cc )
target_link_libraries(object_tracking_data_test ${OpenCV_LIBRARIES})

catkin_add_gtest( object_ranker_test filters/object_ranker_test.cc )
target_link_libraries(object_ranker_test ${OpenCV_LIBRARIES})

catkin_add_gtest( object_full_data_test filters/object_full_data_test.cc )
target_link_libraries(object_full_data_test ${OpenCV_LIBRARIES})

catkin_add_gtest( object_frame_memory_test filters/object_frame_memory_test.cc )
target_link_libraries(object_frame_memory_test ${OpenCV_LIBRARIES})

catkin_add_gtest( object_basic_data_test filters/object_basic_data_test.cc )
target_link_libraries(object_basic_data_test ${OpenCV_LIBRARIES})

catkin_add_gtest( feature_factory_test filters/feature_factory_test.cc )
target_link_libraries(feature_factory_test ${OpenCV_LIBRARIES})

catkin_add_gtest( performace_evaluator_test filters/performace_evaluator_test.cc )
target_link_libraries(performace_evaluator_test ${OpenCV_LIBRARIES})

catkin_add_gtest( contour_test filters/contour_test.cc )
target_link_libraries(contour_test ${OpenCV_LIBRARIES})
