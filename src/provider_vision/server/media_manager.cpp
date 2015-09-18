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
MediaManager::MediaManager(atlas::NodeHandlePtr node_handle)
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
std::vector<std::string> MediaManager::GetCameraList() const {
  std::vector<std::string> cameraList;

  for (auto &elem : context_) {
    std::vector<std::string> context_camera_list = elem->GetCameraList();
    for (auto &context_camera : context_camera_list) {
      cameraList.push_back(context_camera);
    }
  }
  return cameraList;
}

//------------------------------------------------------------------------------
//
void MediaManager::StreammingCmd(Command cmd, const std::string &mediaName,
                                 std::shared_ptr<MediaStreamer> &ptr) {
  std::shared_ptr<BaseContext> driver = GetDriverForCamera(mediaName);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_MANAGER]", "No driver found for this driver");
    return;
  }

  switch (cmd) {
    case Command::START:
      if (driver->StartCamera(mediaName)) {
        std::shared_ptr<BaseMedia> media = driver->GetMedia(mediaName);

        if (media.get() != nullptr) {
          ptr = std::make_shared<MediaStreamer>(media, 30);
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
    case Command::STOP:
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

      driver->StopCamera(mediaName);

      break;
    default:
      ROS_WARN_NAMED("[CAM_MANAGER]", "Unrecognize command.");
      break;
  }
}

//=============================================================================
//
void MediaManager::ParametersCmd(Command cmd, const std::string &mediaName,
                                 BaseCamera::Feature feat, float &val) {
  std::shared_ptr<BaseContext> driver = GetDriverForCamera(mediaName);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_TAG]", "No driver found for this driver");
    return;
  }
  switch (cmd) {
    case Command::SET_FEATURE:
      driver->SetFeature(feat, mediaName, val);
      break;
    case Command::GET_FEATURE:
      driver->GetFeature(feat, mediaName, val);
      break;
    default:
      ROS_WARN_NAMED("[CAM_MANAGER]", "Unrecognize command");
      break;
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::InitializeContext() {
  std::stringstream ss;
  ss << kConfigPath << "/camera_config.xml";
  ConfigurationHandler configHandler(ss.str());
  std::vector<CameraConfiguration> list
    = configHandler.ParseConfiguration();
  // Each time you have a new driver (Gige, usb, etc.) add it to
  // the list here.
  context_.push_back(std::make_shared<DC1394Context>());
  context_.push_back(std::make_shared<WebcamContext>());
  context_.push_back(std::make_shared<VideoFileContext>());

  for (auto &elem : context_) {
    elem->InitContext(list);
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::CloseContext() {
  // Close every devices here
  for (auto &elem : context_) {
    elem->CloseContext();
  }
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
BaseCamera::Feature MediaManager::NameToEnum(const std::string &name) const {
  BaseCamera::Feature to_return = BaseCamera::Feature::ERROR_FEATURE;
  if (name == "SHUTTER_AUTO") {
    to_return = BaseCamera::Feature::SHUTTER_AUTO;
  } else if (name == "SHUTTER") {
    to_return = BaseCamera::Feature::SHUTTER;
  } else if (name == "WHITE_BALANCE_AUTO") {
    to_return = BaseCamera::Feature::WHITE_BALANCE_AUTO;
  } else if (name == "WHITE_BALANCE_RED") {
    to_return = BaseCamera::Feature::WHITE_BALANCE_RED;
  } else if (name == "WHITE_BALANCE_BLUE") {
    to_return = BaseCamera::Feature::WHITE_BALANCE_BLUE;
  } else if (name == "FRAMERATE") {
    to_return = BaseCamera::Feature::FRAMERATE;
  } else {
    ROS_WARN_NAMED("[CAMERA_MANAGER]", "No feature named: %s", name.c_str());
  }
  return to_return;
}

}  // namespace vision_server
