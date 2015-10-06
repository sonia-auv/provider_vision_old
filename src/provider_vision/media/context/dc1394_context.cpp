/**
 * \file	DC1394Context.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <vector>
#include <ros/ros.h>
#include "provider_vision/media/context/dc1394_context.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DC1394Context::DC1394Context() noexcept : BaseContext(),
                                          DRIVER_TAG("[DC1394 Driver]"),
                                          _context(nullptr) {}

//------------------------------------------------------------------------------
//
DC1394Context::~DC1394Context() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DC1394Context::InitContext(
    const std::vector<CameraConfiguration> &cam_configuration_lists) {
  _context = dc1394_new();
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing DC1394 context");

  dc1394error_t error;
  dc1394camera_list_t *list;
  media_list_.clear();

  error = dc1394_camera_enumerate(_context, &list);
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
        camera_dc = dc1394_camera_new(_context, list->ids[i].guid);
        if (camera_dc == nullptr) {
          throw std::runtime_error("Error creating the DC1394 camera");
        }

        std::shared_ptr<DC1394Camera> cam(
            new DC1394Camera(camera_dc, cam_config));

        if (!cam->Open()) {
          throw std::runtime_error("Error opening the camera");
        }

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
    if (!cam->Stop()) {
      throw std::runtime_error("Cannot stop camera");
    }

    if (!cam->Close()) {
      throw std::runtime_error("Cannot close camera");
    }
  }

  media_list_.clear();

  dc1394_free(_context);
}

//------------------------------------------------------------------------------
//
bool DC1394Context::StartCamera(const std::string &name) {
  bool result = false;

  DC1394Camera::Ptr cam = GetDC1394Camera(name);

  if (cam->Start()) {
    result = true;
  } else {
    ROS_ERROR_NAMED(DRIVER_TAG, " Camera opened but not started started ");
  }
  return result;
}

//------------------------------------------------------------------------------
//
bool DC1394Context::StopCamera(const std::string &name) {
  bool result = false;
  DC1394Camera::Ptr cam = GetDC1394Camera(name);

  if (cam->Stop()) {
    result = true;
  } else {
    ROS_ERROR_NAMED(DRIVER_TAG, "Camera opened but not started started ");
  }

  return result;
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

  if (!cam->SetFeature(feat, val)) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Feature setting failed.");
  }
}

//------------------------------------------------------------------------------
//
void DC1394Context::GetFeature(BaseCamera::Feature feat,
                               const std::string &name, float &val) const {
  DC1394Camera::Ptr cam = GetDC1394Camera(name);

  val = cam->GetFeature(feat);
}

}  // namespace vision_server
