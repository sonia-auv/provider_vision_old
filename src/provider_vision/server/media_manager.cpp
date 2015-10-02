/**
 * \file	CameraManager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <stdexcept>
#include <string>
#include <vector>
#include "provider_vision/server/media_manager.h"
#include "provider_vision/media/context/dc1394_context.h"
#include "provider_vision/media/context/webcam_context.h"
#include "provider_vision/media/context/file_context.h"

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
void MediaManager::StartMedia(const std::string &media_name) noexcept {
  GetMedia(media_name)->Start();
}

//------------------------------------------------------------------------------
//
void MediaManager::StopMedia(const std::string &media) noexcept {
  GetMedia(media)->Stop();
}

//------------------------------------------------------------------------------
//
void MediaManager::SetCameraFeature(const std::string &media_name,
                                    const std::string &feature,
                                    float value) noexcept {
  auto camera = dynamic_cast<BaseCamera *>(GetMedia(media_name).get());

  if (camera != nullptr) {
    camera->SetFeature(GetFeatureFromName(feature), value);
  }
  throw std::invalid_argument("The given media is not a camera.");
}

//------------------------------------------------------------------------------
//
float MediaManager::GetCameraFeature(const std::string &media_name,
                                     const std::string &feature) noexcept {
  auto camera = dynamic_cast<BaseCamera *>(GetMedia(media_name).get());

  if (camera != nullptr) {
    return camera->GetFeature(GetFeatureFromName(feature));
  }
  throw std::invalid_argument("The given media is not a camera.");
}

//------------------------------------------------------------------------------
//
BaseMedia::Ptr MediaManager::GetMedia(const std::string &name) const noexcept {
  for (const auto &context : contexts_) {
    if (context->GetMedia(name)->GetName() == name) {
    }
  }
}

//------------------------------------------------------------------------------
//
std::vector<BaseMedia::Ptr> MediaManager::GetAllMedias() const {
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
                                 MediaStreamer::Ptr &ptr) {
  BaseContext::Ptr driver = GetContextFromMedia(media_name);
  // FEATURE* feat = static_cast<FEATURE*>(specific_to_cmd);
  // float* val = static_cast<float*>(specific_to_cmd2);
  if (driver == nullptr) {
    ROS_ERROR_NAMED("[CAM_MANAGER]", "No driver found for this driver");
    return;
  }

  switch (cmd) {
    case Command::START:
      if (driver->StartCamera(media_name)) {
        BaseMedia::Ptr media = driver->GetMedia(media_name);

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
  BaseContext::Ptr driver = GetContextFromMedia(media_name);
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
  contexts_.push_back(std::make_shared<FileContext>());

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
BaseContext::Ptr MediaManager::GetContextFromMedia(
    const std::string &name) const {
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
