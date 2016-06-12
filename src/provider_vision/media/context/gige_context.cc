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

#include <lib_atlas/macros.h>
#ifndef OS_DARWIN

#include <gevapi.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "provider_vision/media/context/dc1394_context.h"
#include "provider_vision/media/context/gige_context.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N
//------------------------------------------------------------------------------
//
GigeContext::GigeContext(
    const std::vector<CameraConfiguration> &configurations) noexcept
    : BaseContext(),
      DRIVER_TAG("[GigE Driver]") {
  InitContext(configurations);
}

//------------------------------------------------------------------------------
//

GigeContext::~GigeContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void GigeContext::InitContext(
    const std::vector<CameraConfiguration> &configurations) {
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing GigE driver");
  UINT16 status;
  int numCamera = 0;

  media_list_.clear();

  status = GevGetCameraList(driver_, MAX_CAMERAS, &numCamera);

  if (status != GEVLIB_OK) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Could not enumerate GigE cameras.");
    return;
  }

  if (numCamera == 0) {
    ROS_WARN_NAMED(DRIVER_TAG, "No GigE camera found.");
    return;
  }
  ROS_INFO_NAMED(DRIVER_TAG, "%d GigE camera found", numCamera);
  for (int i = 0; i < numCamera; i++) {
    std::string name = driver_[i].username;
    for (auto const &cam_config : configurations) {
      if (cam_config.name_ == name) {
        std::shared_ptr<GigeCamera> cam(new GigeCamera(cam_config));

        cam->Open();
        // cam->SetCameraParams();
        media_list_.push_back(cam);
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void GigeContext::CloseContext() {
  for (auto &media : media_list_) {
    GigeCamera::Ptr cam = GetGigeCamera(media);

    if (cam) {
      cam->StopStreaming();
      cam->Close();
    }
  }
  media_list_.clear();
  GevApiUninitialize();
}

//------------------------------------------------------------------------------
//
bool GigeContext::OpenMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  if (cam) {
    return cam->Open();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeContext::CloseMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  if (cam) {
    return cam->Close();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeContext::StartStreamingMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  if (cam) {
    return cam->StartStreaming();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeContext::StopStreamingMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  if (cam) {
    return cam->StopStreaming();
  }
  return false;
}

//------------------------------------------------------------------------------
//
void GigeContext::Run() {
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
bool GigeContext::WatchDogFunc() {
  bool fail = false;

  for (auto &active_camera : media_list_) {
    GigeCamera::Ptr cam = GetGigeCamera(active_camera);

    if (cam->GetAcquistionTimerValue() > TIME_FOR_BUS_ERROR) {
      ROS_FATAL_NAMED(DRIVER_TAG, "Camera is not feeding");
      fail = true;
    }
  }
  return fail;
}

//------------------------------------------------------------------------------
//
bool GigeContext::SetFeature(const BaseCamera::Feature &feat,
                             const std::string &name, const boost::any &val) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  if (cam) {
    return cam->SetFeature(feat, val);
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeContext::GetFeature(const BaseCamera::Feature &feat,
                             const std::string &name, boost::any &val) const {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  if (cam) {
    return cam->GetFeature(feat, val);
  }
  return false;
}

}  // namespace provider_vision

#endif  // OS_DARWIN
