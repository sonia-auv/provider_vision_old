/**
 * \file	AcquisitionLoop.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/media_streamer.h"

namespace vision_server {

const std::string MediaStreamer::LOOP_TAG = "[MediaStreamer]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaStreamer::MediaStreamer(BaseMedia::Ptr cam, int artificialFrameRateMs)
    : is_streaming_(false),
      media_(cam),
      artificial_framerate_(artificialFrameRateMs),
      // here 30 in case we forget to set it, so the cpu doesn't blow off.
      framerate_mili_sec_(1000 / 30),
      image_(),
      video_writer_(),
      is_recording_(false) {
  if (media_->HasArtificialFramerate() && artificial_framerate_ != 0) {
    framerate_mili_sec_ = 1000 / artificial_framerate_;
  }
}

//------------------------------------------------------------------------------
//
MediaStreamer::~MediaStreamer() {
  if (is_streaming_) {
    StopStreaming();
  }
  ROS_INFO_NAMED(LOOP_TAG, "Destroying MediaStreamer");
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MediaStreamer::SetFramerate(int framePerSecond) {
  framerate_mili_sec_ = 1000 / framePerSecond;
}

//------------------------------------------------------------------------------
//
void MediaStreamer::StartStreaming() {
  ROS_INFO_NAMED(LOOP_TAG, "Starting streaming on camera");

  // Start thread
  std::lock_guard<std::mutex> guard(image_access_);
  Start();
  if (IsRunning()) {
    is_streaming_ = true;
  }
}

//------------------------------------------------------------------------------
//
void MediaStreamer::StopStreaming() {
  if (IsStreaming()) {
    is_streaming_ = false;
    // Send message on the line.
    ROS_INFO_NAMED(LOOP_TAG, "Stopping streaming on camera");
    // Stop thread
    StopRecording();

    if (IsRunning()) {
      Stop();
      is_streaming_ = true;
    } else {
      ROS_WARN_NAMED(LOOP_TAG, "Thread is not alive");
    }
  }
}

//------------------------------------------------------------------------------
//
void MediaStreamer::StartRecording(const std::string &filename) {
  if (IsRecording()) {
    return;
  }

  std::string filepath = filename;
  // If no filename was provided, set the default filepath
  if (filepath.empty()) {
    // TODO Thibaut Mattio: Save the video with the current timer.
    filepath = atlas::kLogPath + "save_video" + ".avi";
    ROS_INFO_NAMED(LOOP_TAG, "Starting video on %s", filepath.c_str());
  }

  // mjpg et divx fonctionnels aussi, HFYU est en lossless
  video_writer_ =
      cv::VideoWriter(filepath, CV_FOURCC('D', 'I', 'V', 'X'), 15.0,
                      cv::Size(image_.size().width, image_.size().height));

  if (!video_writer_.isOpened()) {
    ROS_ERROR_NAMED(LOOP_TAG, "Video writer was not opened!");
  } else {
    is_recording_ = true;
  }
}

//------------------------------------------------------------------------------
//
void MediaStreamer::StopRecording() {
  if (IsRecording()) {
    video_writer_.release();
    is_recording_ = false;
  }
}

//------------------------------------------------------------------------------
//
void MediaStreamer::Run() {
  bool must_set_record = false;

  // If the media is a real camera, start recording the feed.
  if (!media_->HasArtificialFramerate()) {
    must_set_record = true;
  }

  while (!MustStop()) {
    // Adding this here, because if the thread has been close,
    // but we have not pass through the while yet, we want to check that...
    if (IsStreaming()) {
      image_access_.lock();
      media_->NextImage(image_);
      image_access_.unlock();

      if (!image_.empty()) {
        // This should not be the proper way to do this.
        // Actually the media, or even better the camera driver sould provide a
        // way
        // to know the size of the video feed they (are going to) send.
        // But for testing purpose we won't do such a modification.
        if (must_set_record) {
          // StartRecording();
          must_set_record = false;
        }

        if (IsRecording() && atlas::PercentageUsedPhysicalMemory() < .8) {
          video_writer_.write(image_);
        }
        Notify();
      }
      atlas::MilliTimer::Sleep(framerate_mili_sec_);
    }
  }

  if (IsRecording()) {
    StopRecording();
  }
}

//------------------------------------------------------------------------------
//
void MediaStreamer::GetImage(cv::Mat &image) const {
  // The user is not supposed to call GetImage if it is streaming.
  // Handling it for compatibility, but this feature is deprecated.
  // TODO Thibaut Mattio: Change the media streamer to send images
  // on notification and remove streaming call to GetImage().
  if (IsStreaming()) {
    std::lock_guard<std::mutex> guard(image_access_);

    try {
      if (image_.empty()) {
        throw std::runtime_error("The media has returned an empty image.");
      } else {
        image = image_;
      }
    } catch (cv::Exception &e) {
      ROS_ERROR_NAMED(LOOP_TAG, "Exception in cloning (%s)", e.what());
      image = cv::Mat::zeros(100, 100, CV_8UC3);
      throw;
    }
  } else {
    image_access_.lock();
    media_->NextImage(image);
    image_access_.unlock();
    if (image.empty()) {
      throw std::runtime_error("The media has returned an empty image.");
    }
  }
}

//------------------------------------------------------------------------------
//
BaseMedia::Status MediaStreamer::GetMediaStatus() const {
  return media_->GetStatus();
}

}  // namespace vision_server
