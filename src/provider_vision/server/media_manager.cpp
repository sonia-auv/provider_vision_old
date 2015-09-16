/**
 * \file	CameraManager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <assert.h>
#include <lib_atlas/config.h>
#include "provider_vision/server/media_manager.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
MediaManager::MediaManager()
    : atlas::ServiceServerManager<MediaManager>(node_handle) {
  assert(node_handle.get() != nullptr);
  std::string base_node_name(kRosNodeName);

  InitializeContext();
};

//------------------------------------------------------------------------------
//
MediaManager::~MediaManager() { CloseContext(); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
std::vector<CameraID> MediaManager::GetCameraList() {
  std::vector<CameraID> cameraList;
  for (auto &elem : context_) {
    std::vector<uint64_t> driver_list = elem->GetCameraList();
    for (auto &driver_list_j : driver_list) {
      cameraList.push_back(driver_list_j);
    }
  }
  return cameraList;
}

//------------------------------------------------------------------------------
//
void MediaManager::StreammingCmd(COMMAND cmd, const std::string &mediaName,
                                 std::shared_ptr<MediaStreamer> &ptr) {
  std::shared_ptr<BaseContext> driver = GetDriverForCamera(mediaName);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_MANAGER]", "No driver found for this driver");
    return;
  }
  CameraID camToStart = driver->GetIDFromName(mediaName);
  switch (cmd) {
    case START:
      if (driver->StartCamera(camToStart)) {
        std::shared_ptr<Media> media_ptr = driver->GetActiveCamera(camToStart);
        if (media_ptr.get() != nullptr) {
          ptr = std::make_shared<MediaStreamer>(media_ptr, 30);
          ptr->StartStreaming();
        } else {
          ptr = nullptr;
          ROS_INFO_NAMED("[CAM_MANAGER]",
                         "Start returned true, but not active camera.");
        }
      } else {
        ptr = nullptr;
        ROS_INFO_NAMED("[CAM_MANAGER]",
                       "MediaStreamer ptr is null! Can't start it!");
      }
      break;
    case STOP:
      if (ptr == nullptr) {
        ROS_ERROR_NAMED("[CAM_MANAGER]",
                        "MediaStreamer ptr is null! Can't stop it!");
        return;
      }
      if (!ptr->StopStreaming()) {
        ROS_ERROR_NAMED("[CAM_MANAGER]",
                        "Error stopping streaming acquisition loop.");
        return;
      }

      driver->StopCamera(camToStart);

      break;
    default:
      ROS_WARN_NAMED("[CAM_MANAGER]", "Unrecognize command.");
      break;
  }
}

//=============================================================================
//
void MediaManager::ParametersCmd(COMMAND cmd, const std::string &mediaName,
                                 FEATURE feat, float &val) {
  std::shared_ptr<BaseContext> driver = GetDriverForCamera(mediaName);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_TAG]", "No driver found for this driver");
    return;
  }
  CameraID cam = driver->GetIDFromName(mediaName);

  switch (cmd) {
    case SET_FEATURE:
      driver->SetFeature(feat, cam, val);
      break;
    case GET_FEATURE:
      driver->GetFeature(feat, cam, val);
      break;
    default:
      ROS_WARN_NAMED("[CAM_MANAGER]", "Unrecognize command");
      break;
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::InitializeContext() {
  // Each time you have a new driver (Gige, usb, etc.) add it to
  // the list here.
  context_.push_back(std::make_shared<CAMDriverDC1394>());
  context_.push_back(std::make_shared<WebcamContext>());
  context_.push_back(std::make_shared<VideoFileContext>());

  for (auto &elem : context_) {
    elem->InitDriver();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool MediaManager::CloseContext() {
  // Close every devices here
  for (auto &elem : context_) {
    elem->CloseDriver();
  }
  return false;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<BaseContext> MediaManager::GetDriverForCamera(
    const std::string &name) {
  for (auto &elem : context_) {
    if (elem->IsMyCamera(name)) {
      return elem;
    }
  }
  ROS_ERROR_NAMED("[CAMERA_MANAGER]",
                  "This camera: %s is not part of any driver instantiated...",
                  name.c_str());
  return nullptr;
}

//------------------------------------------------------------------------------
//
FEATURE MediaManager::NameToEnum(const std::string &name) {
  FEATURE to_return = ERROR_FEATURE;
  if (name == "SHUTTER_AUTO") {
    to_return = SHUTTER_AUTO;
  } else if (name == "SHUTTER") {
    to_return = SHUTTER;
  } else if (name == "WHITE_BALANCE_AUTO") {
    to_return = WHITE_BALANCE_AUTO;
  } else if (name == "WHITE_BALANCE_RED") {
    to_return = WHITE_BALANCE_RED;
  } else if (name == "WHITE_BALANCE_BLUE") {
    to_return = WHITE_BALANCE_BLUE;
  } else if (name == "FRAMERATE") {
    to_return = FRAMERATE;
  } else {
    ROS_WARN_NAMED("[CAMERA_MANAGER]", "No feature named: %s", name.c_str());
  }
  return to_return;
}

}  // namespace vision_server
