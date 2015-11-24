/**
 * \file  detection_task_manager.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  22/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <iostream>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "provider_vision/utils/config.h"
#include "provider_vision/server/detection_task_manager.h"
#include "provider_vision/server/media_manager.h"

auto mmgr = provider_vision::MediaManager();

std::stringstream ss;
ss << provider_vision::kProjectPath << "test/test_image.png";
auto streamer = mmgr.StartStreamingMedia(ss.str());

TEST(DetectionTaskManager, start_detection) {
  auto dmgr = provider_vision::DetectionTaskManager();
  dmgr.StartDetectionTask(streamer, "image_feed", "test");
  ASSERT_TRUE(dmgr.GetAllDetectionTasksName().size() == 1);
  ASSERT_TRUE(*(dmgr.GetAllDetectionTasksName().begin()) == "test");
  ASSERT_TRUE();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
