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

const char *MediaStreamer::LOOP_TAG = "[MediaStreamer]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaStreamer::MediaStreamer(BaseMedia::Ptr cam, int artificialFrameRateMs)
    : media_(cam),
      // here 30 in case we forget to set it, so the cpu doesn't blow off.
      image_() {}

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
bool MediaStreamer::StartStreaming() {
  ROS_INFO_NAMED(LOOP_TAG, "Starting streaming on camera");
  try {
    Start();
  } catch (std::exception &e) {
    ROS_ERROR_NAMED(LOOP_TAG, "Error while starting the thread: %s", e.what());
    return false;
  };

  // Let the thread start.
  atlas::MilliTimer::Sleep(1);
  if (!IsRunning()) {
    ROS_ERROR_NAMED(LOOP_TAG, "Thread could not be started");
    return false;
  }
  if (IsStreaming()) {
    ROS_WARN_NAMED(LOOP_TAG,
                   "Weird, the media is already streaming on startup...");
    // The goal is still to start the media, so if it streams, we are happy...
    return true;
  }
  return media_->StartStreaming();
}

//------------------------------------------------------------------------------
//
bool MediaStreamer::StopStreaming() {
  bool result = false;
  if (IsStreaming()) {
    ROS_INFO("Stopping streaming on camera");
    result = media_->StopStreaming();
  } else {
    ROS_WARN_NAMED(LOOP_TAG,
                   "Weird, the media is not streaming upon closing...");
  }
  try {
    Stop();
  } catch (std::exception &e) {
    ROS_ERROR_NAMED(LOOP_TAG, "Error while stopping thread: %s", e.what());
    result = false;
  };
  return result;
}

//------------------------------------------------------------------------------
//
void MediaStreamer::Run() {
  // Starting a timer for timing the acquisition of the image from the media.
  atlas::MilliTimer timer;
  timer.Start();
  bool result = false;
  while (!MustStop()) {
    if (IsStreaming()) {
      image_access_.lock();
      result = media_->NextImage(image_);
      image_access_.unlock();

      // We gotta a image
      if (!image_.empty() && result) {
        timer.Reset();
        Notify(image_);
      } else {  // Image getting failed
        // If we have been waiting too long for an image, we consider a failure.
        if (timer.MilliSeconds() > 100) {
          StopStreaming();
          ROS_ERROR_NAMED(
              LOOP_TAG,
              "Cannot access the image from the media. Reached timeout.");
          try {
            Stop();
          } catch (std::exception &e) {
            ROS_ERROR_NAMED(LOOP_TAG, "Error while stopping thread: %s",
                            e.what());
          };
          return;
        }
      }
      // For the files, we set a fixed framerate at 15 fps
      if (media_->HasArtificialFramerate()) {
        atlas::MilliTimer::Sleep(1000 / ARTIFICIAL_FRAMERATE);
      }
    }
  }
}

//------------------------------------------------------------------------------
//
BaseMedia::Status MediaStreamer::GetMediaStatus() const {
  return media_->GetStatus();
}

}  // namespace provider_vision
