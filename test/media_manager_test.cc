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
#include <lib_atlas/sys/timer.h>
#include "provider_vision/utils/config.h"
#include "provider_vision/server/media_manager.h"

using namespace vision_server;

TEST(MediaManagerTest, webcam) {
  MediaManager mmng;

  // Assert that there is a webcam object in the system.
  // If not, just do nothing.
  std::vector<std::string> names = mmng.GetAllMediasName();
  if(std::find(names.begin(), names.end(), "Webcam") != names.end()) {
    // The medias that has been created is streaming.
    ASSERT_EQ(mmng.GetMediaStatus("Webcam"), BaseMedia::Status::CLOSE);

    mmng.OpenMedia("Webcam");
    ASSERT_EQ(mmng.GetMediaStatus("Webcam"), BaseMedia::Status::OPEN);

    // We are able to get the media streamer from the newly created media.
    MediaStreamer::Ptr mstreamer = mmng.StartStreamingMedia("Webcam");
    ASSERT_NE(mstreamer.get(), nullptr);

    // The media streamer is directing to the correct media.
    ASSERT_EQ(mstreamer->GetMediaName().compare("Webcam"), 0);

    // The medias that has been created is streaming.
    auto test = mstreamer->GetMediaStatus();
    ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::STREAMING);

    // Waiting for the first image from the webcam..
    atlas::MilliTimer::Sleep(100);

    // We cam acquire an image from the media.
    cv::Mat image;
    mstreamer->GetImage(image);
    ASSERT_EQ(image.empty(), false);

    // We can close the media.
    mmng.StopStreamingMedia("Webcam");
    ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::OPEN);

    // Trying to get an image again. This should work as we did not closed
    // the media.
    mstreamer->GetImage(image);
    ASSERT_EQ(image.empty(), false);

    // We can close the media.
    mmng.CloseMedia("Webcam");
    ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::CLOSE);

    // If the media is closed, the system throw an exception
    ASSERT_THROW(mstreamer->GetImage(image), std::runtime_error);
    ASSERT_TRUE(image.empty());
  }
}

TEST(MediaManagerTest, image) {
  MediaManager mmng;

  std::string file_path(kProjectPath + "test/test_image.png");
  auto size_init = mmng.GetAllMediasCount();
  auto size_after = mmng.GetAllMediasCount();

  // There is one and only one media that has been created in the system
  ASSERT_NE(size_init +1, size_after);

  // We are able to get the media streamer from the newly created media.
  MediaStreamer::Ptr mstreamer = mmng.StartStreamingMedia(file_path);
  ASSERT_NE(mstreamer.get(), nullptr);

  // The media streamer is directing to the correct media.
  ASSERT_EQ(mstreamer->GetMediaName().compare(file_path), 0);

  // The medias that has been created is streaming.
  ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::STREAMING);

  // We cam acquire an image from the media.
  cv::Mat image;
  mstreamer->GetImage(image);
  ASSERT_EQ(image.empty(), false);

  // We can close the media.
  mmng.StopStreamingMedia(file_path);
  ASSERT_EQ(mstreamer->GetMediaStatus(), BaseMedia::Status::CLOSE);

  // After we closed the media, it still exists in the system.
  ASSERT_NE(size_init +1, size_after);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
