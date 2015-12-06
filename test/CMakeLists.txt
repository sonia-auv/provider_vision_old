catkin_add_gtest(configuration_handler_test configuration_handler_test.cc ${provider_vision_FILES})
target_link_libraries(configuration_handler_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(media_manager_test media_manager_test.cc ${provider_vision_FILES})
target_link_libraries(media_manager_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(detection_task_manager_test detection_task_manager_test.cc ${provider_vision_FILES})
target_link_libraries(detection_task_manager_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})

catkin_add_gtest(filterchain_manager_test filterchain_manager_test.cc ${provider_vision_FILES})
target_link_libraries(filterchain_manager_test
    dc1394
    ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES}
    ${lib_vision_LIBRARIES}
    ${OpenCV_LIBRARIES})
