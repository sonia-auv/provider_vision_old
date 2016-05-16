/**
 * \file  camera_calibration.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date  15/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "provider_vision/server/media_manager.h"

ros::NodeHandle *nhp;

TEST(CameraCalibration, CreateGraph) {
  provider_vision::MediaManager mmng(*nhp);

  mmng.OpenMedia("Webcam");
  ASSERT_EQ(mmng.GetMediaStatus("Webcam"), provider_vision::BaseMedia::Status::OPEN);

  // We are able to get the media streamer from the newly created media.
  provider_vision::MediaStreamer::Ptr mstreamer = mmng.StartStreamingMedia("Webcam");
  ASSERT_NE(mstreamer.get(), nullptr);

  // The media streamer is directing to the correct media.
  ASSERT_EQ(mstreamer->GetMediaName().compare("Webcam"), 0);

  // The medias that has been created is streaming.
  ASSERT_EQ(mstreamer->GetMediaStatus(), provider_vision::BaseMedia::Status::STREAMING);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
