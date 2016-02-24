/**
 * \file	dc1394_context.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <string>
#include <vector>
#include <ros/ros.h>
#include "provider_vision/media/context/dc1394_context.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DC1394Context::DC1394Context() noexcept : BaseContext(),
                                          DRIVER_TAG("[DC1394 Driver]"),
                                          driver_(nullptr) {}

//------------------------------------------------------------------------------
//
DC1394Context::~DC1394Context() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DC1394Context::InitContext(
    const std::vector<CameraConfiguration> &cam_configuration_lists) {
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

  dc1394camera_t *camera_dc;
  for (uint i = 0; i < list->num; i++) {
    for (auto const &cam_config : cam_configuration_lists) {
      if (cam_config.GetGUID() == list->ids[i].guid) {
        camera_dc = dc1394_camera_new(driver_, list->ids[i].guid);
        if (camera_dc == nullptr) {
          throw std::runtime_error("Error creating the DC1394 camera");
        }

        std::shared_ptr<DC1394Camera> cam(
            new DC1394Camera(camera_dc, cam_config));

        cam->Open();
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
void DC1394Context::SetFeature(BaseCamera::Feature feat,
                               const std::string &name, float val) {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  cam->SetFeature(feat, val);
}

//------------------------------------------------------------------------------
//
void DC1394Context::GetFeature(BaseCamera::Feature feat,
                               const std::string &name, float &val) const {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);
  val = cam->GetFeature(feat);
}

}  // namespace provider_vision
