/**
 * \file	AcquisitionLoop.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <ros/ros.h>
#include <lib_atlas/sys/timer.h>
#include <lib_atlas/sys/fsinfo.h>
#include "provider_vision/media/media_streamer.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
MediaStreamer::MediaStreamer(std::shared_ptr<BaseMedia> cam,
                             int artificialFrameRateMs)
    : media_(cam),
      LOOP_TAG("[Acquisition Loop]"),
      is_streaming_(false),
      artificial_framerate_(artificialFrameRateMs),
      // here 30 in case we forget to set it, so the cpu doesn't blow off.
      framerate_mili_sec_(1000 / 30),
      image_(),
      video_writer_(),
      is_recording_(false) {
  if (media_->HasArtificialFramerate() && artificial_framerate_ != 0)
    framerate_mili_sec_ = 1000 / artificial_framerate_;
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
bool MediaStreamer::StartStreaming() {
  ROS_INFO_NAMED(LOOP_TAG, "Starting streaming on camera %s",
                 media_->GetName());

  // Start thread
  std::lock_guard<std::mutex> guard(image_access_);
  start();
  if (running()) {
    is_streaming_ = true;
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool MediaStreamer::StopStreaming() {
  is_streaming_ = false;

  // Send message on the line.
  ROS_INFO_NAMED(LOOP_TAG, "Stopping streaming on camera %s",
                 media_->GetName());
  // Stop thread
  if (running()) {
    stop();
    return true;
  } else {
    ROS_WARN_NAMED(LOOP_TAG, "Thread is not alive");
  }
}

//------------------------------------------------------------------------------
//
bool MediaStreamer::StartRecording(const std::string &filename) {
  if (IsRecording()) {
    // ROS_INFO("[VISION_CLIENT] startVideoCapture not opened.");
    return false;
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
    return false;
  }
  is_recording_ = true;
  return true;
}

//------------------------------------------------------------------------------
//
bool MediaStreamer::StopRecording() {
  if (IsRecording()) {
    video_writer_.release();
    is_recording_ = false;
    return true;
  }
  ROS_INFO("[VISION_CLIENT] stopVideoCapture video is not running.");
  return false;
}

//------------------------------------------------------------------------------
//
void MediaStreamer::run() {
  bool acquival = false;
  bool must_set_record = false;

  // If the media is a real camera, start recording the feed.
  if (!media_->HasArtificialFramerate()) {
    must_set_record = true;
  }

  while (!must_stop()) {
    //_logger->LogInfo(LOOP_TAG, "Taking mutex for publishing");
    image_access_.lock();
    //_logger->LogInfo(LOOP_TAG, "Took mutex for publishing");
    acquival = media_->NextImage(image_);
    image_access_.unlock();
    //_logger->LogInfo(LOOP_TAG, "Releasing mutex for publishing");

    if (!acquival) {
      image_ = cv::Mat::zeros(100, 100, CV_8UC3);
      ROS_ERROR_NAMED(LOOP_TAG, "Error on NextImage. Providing empty image %s",
                      media_->GetName());
    } else {
      // This should not be the proper way to do this.
      // Actually the media, or even better the camera driver sould provide a
      // way
      // to know the size of the video feed they (are going to) send.
      // But for testing purpose we won't do such a modification.
      if (must_set_record) {
        // StartRecording();
        must_set_record = false;
      }

      if (IsRecording() && atlas::percentage_used_physical_memory() < .8) {
        //        cv::Mat image_with_correct_format;
        //        cv::cvtColor(_image, image_with_correct_format, CV_BGR2RGB);
        //        video_writer_.write(image_with_correct_format);
        video_writer_.write(image_);
      }

      // Adding this here, because if the thread has been close,
      // but we have not pass through the while yet, we want to check that...
      if (IsStreaming()) {
        Notify();
      }
    }

    atlas::MilliTimer::sleep(_frameRateMiliSec);
  }

  if (IsRecording()) {
    StopRecording();
  }
}

//------------------------------------------------------------------------------
//
bool MediaStreamer::GetImage(cv::Mat &image) const {
  bool retval = false;

  //_logger->LogInfo(LOOP_TAG, "Taking mutex for getting image");
  std::lock_guard<std::mutex> guard(image_access_);
  //_logger->LogInfo(LOOP_TAG, "Took mutex for getting image");
  if (image_.empty()) {
    image = cv::Mat::zeros(100, 100, CV_8UC3);
    ROS_ERROR_NAMED(LOOP_TAG, "Image is empty");
    return retval;
  }

  // Here we clone, since we want the acquisition loop to be the only
  // owner of the image. A copy would also give the memory address.
  try {
    image = image_.clone();
    retval = true;
  } catch (cv::Exception &e) {
    ROS_ERROR_NAMED(LOOP_TAG, "Exception in cloning (%s)", e.what());
    image = cv::Mat::zeros(100, 100, CV_8UC3);
    retval = false;
  };
  return retval;
}

//------------------------------------------------------------------------------
//
BaseMedia::Status MediaStreamer::GetMediaStatus() const {
  return media_->GetStatus();
}

}  // namespace vision_server
