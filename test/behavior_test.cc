/**
 * \file  contour_test.cc
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "provider_vision/utils/config.h"
#include "provider_vision/server/media_manager.h"

using namespace vision_server;

TEST(MediaManagerTest, media_manager_test) {
  std::string file_path(kProjectPath + "test/test_image.png");

  MediaManager mmng;
  std::vector<std::string> names = mmng.GetAllMediasName();
  ASSERT_GE(names.size(), 1);
  ASSERT_TRUE(std::find(names.begin(), names.end(), "Webcam") != names.end());

  MediaStreamer::Ptr mstreamer = mmng.StartMedia(file_path);
  ASSERT_NE(mstreamer.get(), nullptr);
  auto test = mstreamer->GetMediaStatus();
  ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::STREAMING);
  ASSERT_EQ(mstreamer->GetMediaName().compare(file_path), 0);

  cv::Mat image;
  mstreamer->GetImage(image);
  ASSERT_EQ(image.empty(), false);

  mmng.StopMedia(file_path);
  ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::CLOSE);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
