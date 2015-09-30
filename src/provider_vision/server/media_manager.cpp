/**
 * \file	CameraManager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_atlas/config.h>
#include "provider_vision/server/media_manager.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaManager::MediaManager() noexcept : contexts_() { InitializeContext(); }

//------------------------------------------------------------------------------
//
MediaManager::~MediaManager() noexcept { CloseContext(); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
std::shared_ptr<MediaStreamer> MediaManager::StartCamera(
    const std::string &media_name) noexcept {
  GetMedia()
}

//------------------------------------------------------------------------------
//
void MediaManager::StopCamera(const std::string &media) noexcept {

}

//------------------------------------------------------------------------------
//
void MediaManager::SetFeature(const std::string &media_name, BaseCamera::Feature feat,
                float val) noexcept {

}

//------------------------------------------------------------------------------
//
float MediaManager::GetFeature(const std::string &media_name,
                 BaseCamera::Feature feat) noexcept {

}

//------------------------------------------------------------------------------
//
std::vector<BaseMedia> MediaManager::GetAllMedias() const {
  std::vector<BaseMedia> medias;

  for (auto &elem : contexts_) {
    for (auto &context : contexts_) {
      medias.push_back(context);
    }
  }
  return medias;
}

//------------------------------------------------------------------------------
//
void MediaManager::StreammingCmd(Command cmd, const std::string &media_name,
                                 std::shared_ptr<MediaStreamer> &ptr) {
  std::shared_ptr<BaseContext> driver = GetContextFromMedia(media_name);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_MANAGER]", "No driver found for this driver");
    return;
  }

  switch (cmd) {
    case Command::START:
      if (driver->StartCamera(media_name)) {
        std::shared_ptr<BaseMedia> media = driver->GetMedia(media_name);

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

      driver->StopCamera(media_name);

      break;
    default:
      ROS_WARN_NAMED("[CAM_MANAGER]", "Unrecognize command.");
      break;
  }
}

//=============================================================================
//
void MediaManager::ParametersCmd(Command cmd, const std::string &media_name,
                                 BaseCamera::Feature feat, float &val) {
  std::shared_ptr<BaseContext> driver = GetContextFromMedia(media_name);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_TAG]", "No driver found for this driver");
    return;
  }
  switch (cmd) {
    case Command::SET_FEATURE:
      driver->SetFeature(feat, media_name, val);
      break;
    case Command::GET_FEATURE:
      driver->GetFeature(feat, media_name, val);
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
  std::vector<CameraConfiguration> list = configHandler.ParseConfiguration();
  // Each time you have a new driver (Gige, usb, etc.) add it to
  // the list here.
  contexts_.push_back(std::make_shared<DC1394Context>());
  contexts_.push_back(std::make_shared<WebcamContext>());
  contexts_.push_back(std::make_shared<VideoFileContext>());

  for (auto &elem : contexts_) {
    elem->InitContext(list);
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::CloseContext() {
  // Close every devices here
  for (auto &elem : contexts_) {
    elem->CloseContext();
  }
}

//------------------------------------------------------------------------------
//
std::shared_ptr<BaseContext> MediaManager::GetContextFromMedia(
    const std::string &name) {
  for (auto &context : contexts_) {
    if (context->ContainsMedia(name)) {
      return context;
    }
  }
  throw std::invalid_argument("No context contains this media");
}

//------------------------------------------------------------------------------
//
BaseCamera::Feature MediaManager::GetFeatureFromName(
    const std::string &name) const {
  if (name == "SHUTTER_AUTO") {
    return BaseCamera::Feature::SHUTTER_AUTO;
  } else if (name == "SHUTTER") {
    return BaseCamera::Feature::SHUTTER;
  } else if (name == "WHITE_BALANCE_AUTO") {
    return BaseCamera::Feature::WHITE_BALANCE_AUTO;
  } else if (name == "WHITE_BALANCE_RED") {
    return BaseCamera::Feature::WHITE_BALANCE_RED;
  } else if (name == "WHITE_BALANCE_BLUE") {
    return BaseCamera::Feature::WHITE_BALANCE_BLUE;
  } else if (name == "FRAMERATE") {
    return BaseCamera::Feature::FRAMERATE;
  }
  throw std::invalid_argument("No feature with this name");
}

}  // namespace vision_server
