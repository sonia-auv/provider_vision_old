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

#include "provider_vision/media/context/dc1394_context.h"
#include <ros/ros.h>
#include <string>
#include <vector>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DC1394Context::DC1394Context(
    const std::vector<CameraConfiguration> &configurations) noexcept
    : BaseContext(), DRIVER_TAG("[DC1394 Driver]"), driver_(nullptr) {
  InitContext(configurations);
}

//------------------------------------------------------------------------------
//
DC1394Context::~DC1394Context() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DC1394Context::InitContext(
    const std::vector<CameraConfiguration> &configurations) {
  driver_ = dc1394_new();
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing DC1394 driver");

  dc1394error_t error;
  dc1394camera_list_t *list;
  media_list_.clear();

  error = dc1394_camera_enumerate(driver_, &list);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Could not enumerate camera.");
    return;
  }

  if (list->num == 0) {
    ROS_WARN_NAMED(DRIVER_TAG, "No camera found.");
    return;
  }
  ROS_INFO_NAMED(DRIVER_TAG, "%d DC1394 camera found", list->num);
  dc1394camera_t *camera_dc;
  for (uint i = 0; i < list->num; i++) {
    for (auto const &cam_config : configurations) {
      if (cam_config.guid_ == list->ids[i].guid) {
        camera_dc = dc1394_camera_new(driver_, list->ids[i].guid);
        if (camera_dc == nullptr) {
          throw std::runtime_error("Error creating the DC1394 camera");
        }

        std::shared_ptr<DC1394Camera> cam(
            new DC1394Camera(camera_dc, cam_config));

        cam->SetCameraParams();
        media_list_.push_back(cam);
      }
    }
  }
  dc1394_camera_free_list(list);
}

//------------------------------------------------------------------------------
//
void DC1394Context::CloseContext() {
  for (auto &media : media_list_) {
    DC1394Camera::Ptr cam = GetDC1394Camera(media);
    // Stop if running
    cam->StopStreaming();

    cam->Close();
  }

  media_list_.clear();

  dc1394_free(driver_);
}

//------------------------------------------------------------------------------
//
void DC1394Context::OpenMedia(const std::string &name) {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->Open();
}

//------------------------------------------------------------------------------
//
void DC1394Context::CloseMedia(const std::string &name) {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->Close();
}

//------------------------------------------------------------------------------
//
void DC1394Context::StartStreamingMedia(const std::string &name) {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->StartStreaming();
}

//------------------------------------------------------------------------------
//
void DC1394Context::StopStreamingMedia(const std::string &name) {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->StopStreaming();
}

//------------------------------------------------------------------------------
//
void DC1394Context::Run() {
  while (!MustStop()) {
    atlas::SecTimer::Sleep(5);
    if (!WatchDogFunc()) {
      ROS_WARN_NAMED(DRIVER_TAG, "Watchdog returned with error");
      atlas::SecTimer::Sleep(2);
    }
  }
}

//------------------------------------------------------------------------------
//
bool DC1394Context::WatchDogFunc() {
  bool fail = false;

  for (auto &active_camera : media_list_) {
    DC1394Camera::Ptr cam = GetDC1394Camera(active_camera);

    if (cam->GetAcquistionTimerValue() > TIME_FOR_BUS_ERROR) {
      ROS_FATAL_NAMED(DRIVER_TAG, "Camera is not feeding!");
      fail = true;
    }
  }
  return fail;
}

//------------------------------------------------------------------------------
//
void DC1394Context::SetFeature(const BaseCamera::Feature &feat,
                               const std::string &name, boost::any &val) {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->SetFeature(feat, val);
}

//------------------------------------------------------------------------------
//
void DC1394Context::GetFeature(const BaseCamera::Feature &feat,
                               const std::string &name, boost::any &val) const {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->GetFeature(feat, val);
}

}  // namespace provider_vision
