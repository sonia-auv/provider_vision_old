/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include "provider_vision/media/media_streamer.h"

namespace provider_vision {

const std::string MediaStreamer::LOOP_TAG = "[MediaStreamer]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaStreamer::MediaStreamer(BaseMedia::Ptr cam, int artificialFrameRateMs)
    : media_(cam),
      artificial_framerate_(artificialFrameRateMs),
      // here 30 in case we forget to set it, so the cpu doesn't blow off.
      framerate_mili_sec_(1000 / 30),
      image_(),
      video_writer_(),
      is_recording_(false) {
  if (media_->HasArtificialFramerate() && artificial_framerate_ != 0) {
    framerate_mili_sec_ = 1000 / artificial_framerate_;
  }
  Start();
}

//------------------------------------------------------------------------------
//
MediaStreamer::~MediaStreamer() {
  if (IsStreaming()) {
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
  if (IsRunning() && !IsStreaming()) {
    media_->StartStreaming();
  }
}

//------------------------------------------------------------------------------
//
void MediaStreamer::StopStreaming() {
  if (IsRecording()) {
    StopRecording();
  }
  if (IsStreaming()) {
    ROS_INFO("Stopping streaming on camera");
    try {
      media_->StopStreaming();
      Stop();
    } catch (std::exception &e) {
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

  // Starting a timer for timing the acquisition of the image from the media.
  atlas::MilliTimer timer;
  timer.Start();

  while (!MustStop()) {
    if (IsStreaming()) {
      image_access_.lock();
      media_->NextImage(image_);
      image_access_.unlock();

      if (!image_.empty()) {
        timer.Reset();
        if (must_set_record) {
          // StartRecording();
          must_set_record = false;
        }

        if (IsRecording() && atlas::PercentageUsedPhysicalMemory() < .8) {
          video_writer_.write(image_);
        }
        Notify(image_);
      } else {
        if (timer.MilliSeconds() > 100) {
          StopStreaming();
          throw std::runtime_error(
              "Cannot access the image from the media. Reached timeout.");
        }
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
  if (!IsStreaming()) {
    image_access_.lock();
    media_->NextImage(image);
    image_access_.unlock();
    if (image.empty()) {
      throw std::runtime_error("The media has returned an empty image.");
    }
  } else {
    throw std::logic_error(
        "Cannot acquire an image from a media that is streaming");
  }
}

//------------------------------------------------------------------------------
//
BaseMedia::Status MediaStreamer::GetMediaStatus() const {
  return media_->GetStatus();
}

}  // namespace provider_vision
