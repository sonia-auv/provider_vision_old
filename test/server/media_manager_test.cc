/**
 * \file  media_manager_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  22/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <lib_atlas/sys/timer.h>
#include "provider_vision/config.h"
#include "provider_vision/server/media_manager.h"

ros::NodeHandle *nhp;

/**
 * Observer class that will store the image object from the media streamer
 * whenever it send a notification.
 */
class ImageObserver : public atlas::Observer<const cv::Mat &> {
 public:
  ImageObserver() noexcept : Observer() {}
  virtual ~ImageObserver() noexcept {}
  cv::Mat image_;

 private:
  void OnSubjectNotify(atlas::Subject<const cv::Mat &> &subject,
                       const cv::Mat &image) noexcept override {
    image_ = image;
  }
};

TEST(MediaManagerTest, webcam) {
  provider_vision::MediaManager mmng(*nhp);

  // Assert that there is a webcam object in the system.
  // If not, just do nothing.
  std::vector<std::string> names = mmng.GetAllMediasName();
  if (std::find(names.begin(), names.end(), "Webcam") != names.end()) {
    mmng.OpenMedia("Webcam");

    // We are able to get the media streamer from the newly created media.
    provider_vision::MediaStreamer::Ptr mstreamer =
        mmng.StartStreamingMedia("Webcam");
    ASSERT_NE(mstreamer.get(), nullptr);

    // The media streamer is directing to the correct media.
    ASSERT_EQ(mstreamer->GetMediaName().compare("Webcam"), 0);

    // The medias that has been created is streaming.
    ASSERT_EQ(mstreamer->GetMediaStatus(),
              provider_vision::BaseMedia::Status::STREAMING);

    // Creating the observer for the image.
    auto image_observer = ImageObserver();
    image_observer.Observe(*mstreamer);

    // Waiting for the first image from the webcam..
    atlas::MilliTimer::Sleep(1000);

    // We cam acquire an image from the media.
    ASSERT_EQ(image_observer.image_.empty(), false);

    // This should throw, not supposed to call GetImage when the media is
    // streaming...
    cv::Mat image;
    ASSERT_THROW(mstreamer->GetImage(image), std::logic_error);

    // We can close the media.
    mmng.StopStreamingMedia("Webcam");
    ASSERT_EQ(mstreamer->GetMediaStatus(),
              provider_vision::BaseMedia::Status::OPEN);

    // Trying to get an image again. This should work as we did not closed
    // the media.
    mstreamer->GetImage(image);
    ASSERT_EQ(image.empty(), false);

    // We can close the media.
    mmng.CloseMedia("Webcam");
    ASSERT_EQ(mstreamer->GetMediaStatus(),
              provider_vision::BaseMedia::Status::CLOSE);

    // If the media is closed, the system throw an exception
    ASSERT_THROW(mstreamer->GetImage(image), std::runtime_error);
    ASSERT_TRUE(image.empty());
  }
}

TEST(MediaManagerTest, image) {
  provider_vision::MediaManager mmng(*nhp);

  // Assert that there is a webcam object in the system.
  // If not, just do nothing.
  std::vector<std::string> names = mmng.GetAllMediasName();

  std::string file_path(provider_vision::kProjectPath +
                        "test/img/test_image.png");
  auto size_init = mmng.GetAllMediasCount();
  auto size_after = mmng.GetAllMediasCount();

  // There is one and only one media that has been created in the system
  ASSERT_NE(size_init + 1, size_after);

  // We are able to get the media streamer from the newly created media.
  mmng.OpenMedia(file_path);

  // We are able to get the media streamer from the newly created media.
  provider_vision::MediaStreamer::Ptr mstreamer =
      mmng.StartStreamingMedia(file_path);
  ASSERT_NE(mstreamer.get(), nullptr);

  // The media streamer is directing to the correct media.
  ASSERT_EQ(mstreamer->GetMediaName().compare(file_path), 0);

  // The medias that has been created is streaming.
  auto test = mstreamer->GetMediaStatus();
  ASSERT_EQ(mstreamer->GetMediaStatus(),
            provider_vision::BaseMedia::Status::STREAMING);

  // Creating the observer for the image.
  auto image_observer = ImageObserver();
  image_observer.Observe(*mstreamer);

  // Waiting for the first image from the webcam..
  atlas::MilliTimer::Sleep(100);

  // We cam acquire an image from the media.
  ASSERT_EQ(image_observer.image_.empty(), false);

  // This should throw, not supposed to call GetImage when the media is
  // streaming...
  cv::Mat image;
  ASSERT_THROW(mstreamer->GetImage(image), std::logic_error);

  // We can close the media.
  mmng.StopStreamingMedia(file_path);
  ASSERT_EQ(mstreamer->GetMediaStatus(),
            provider_vision::BaseMedia::Status::OPEN);

  // Trying to get an image again. This should work as we did not closed
  // the media.
  mstreamer->GetImage(image);
  ASSERT_EQ(image.empty(), false);

  // We can close the media.
  mmng.CloseMedia(file_path);
  ASSERT_EQ(mstreamer->GetMediaStatus(),
            provider_vision::BaseMedia::Status::CLOSE);

  // If the media is closed, the system throw an exception
  ASSERT_THROW(mstreamer->GetImage(image), std::runtime_error);
  ASSERT_TRUE(image.empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
