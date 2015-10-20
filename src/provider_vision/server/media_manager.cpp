/**
 * \file	CameraManager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <stdexcept>
#include <vector>
#include "ros/console.h"
#include "provider_vision/server/media_manager.h"
#include "provider_vision/media/context/dc1394_context.h"
#include "provider_vision/media/context/webcam_context.h"
#include "provider_vision/media/context/file_context.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaManager::MediaManager() noexcept : contexts_() {
  std::stringstream ss;
  ss << kConfigPath << "/camera_config.xml";
  ConfigurationHandler configHandler(ss.str());
  std::vector<CameraConfiguration> list = configHandler.ParseConfiguration();
  // Each time you have a new driver (Gige, usb, etc.) add it to
  // the list here.
  contexts_.push_back(std::make_shared<WebcamContext>());
  contexts_.push_back(std::make_shared<DC1394Context>());
  contexts_.push_back(std::make_shared<FileContext>());

  for (auto &elem : contexts_) {
    elem->InitContext(list);
  }
}

//------------------------------------------------------------------------------
//
MediaManager::~MediaManager() noexcept {
  for (auto &elem : contexts_) {
    elem->CloseContext();
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
MediaStreamer::Ptr MediaManager::StartMedia(const std::string &media_name) {
  MediaStreamer::Ptr streamer(nullptr);
  if (IsMediaStreamerExist(media_name)) {
    streamer = GetMediaStreamer(media_name);
  } else {
    BaseContext::Ptr context = GetContextFromMedia(media_name);
    if (!context) {
      throw std::invalid_argument("The media is not part of a context.");
    }
    context->StartCamera(media_name);
    BaseMedia::Ptr media = context->GetMedia(media_name);

    if (media) {
      streamer = std::make_shared<MediaStreamer>(media, 30);
      streamer->StartStreaming();
      AddMediaStreamer(streamer);
    } else {
      throw std::runtime_error("Camera failed to be created");
    }
  }
  return streamer;
}

//------------------------------------------------------------------------------
//
void MediaManager::StopMedia(const std::string &media) noexcept {
  MediaStreamer::Ptr elem = GetMediaStreamer(media);

  if (elem) {
    // Check to make sure no other task is using this media.
    if (elem->ObserverCount() > 0) {
      return;
    }

    if (elem->IsStreaming()) {
      elem->StopStreaming();
      RemoveMediaStreamer(media);
    } else {
      throw std::invalid_argument("Media is not streaming.");
    }

    BaseMedia::Ptr media_ptr = GetMedia(media);
    if (media_ptr) {
      media_ptr->Stop();
    } else {
      throw std::runtime_error("Media do now exist");
    }
  }
}

//------------------------------------------------------------------------------
//
BaseMedia::Ptr MediaManager::GetMedia(const std::string &name) const noexcept {
  BaseMedia::Ptr media(nullptr);
  for (const auto &context : contexts_) {
    if (context->ContainsMedia(name)) {
      media = context->GetMedia(name);
    }
  }
  return media;
}

//------------------------------------------------------------------------------
//
std::vector<std::string> MediaManager::GetAllMediasName() const noexcept {
  std::vector<std::string> medias;

  for (auto &context : contexts_) {
    auto context_media_list = context->GetMediaList();
    for (const auto &media : context_media_list) {
      medias.push_back(media->GetName());
    }
  }
  return medias;
}

//------------------------------------------------------------------------------
//
void MediaManager::SetCameraFeature(const std::string &media_name,
                                    const std::string &feature, float value) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context) {
    context->SetFeature(GetFeatureFromName(feature), media_name, value);
  } else {
    throw std::invalid_argument("Context not found");
  }
}

//------------------------------------------------------------------------------
//
float MediaManager::GetCameraFeature(const std::string &media_name,
                                     const std::string &feature) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  float value = -1.0f;
  if (context) {
    context->GetFeature(GetFeatureFromName(feature), media_name, value);
  } else {
    throw std::invalid_argument("Context not found");
  }
  return value;
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
  } else {
    throw std::invalid_argument("No feature with this name");
  }
}

}  // namespace vision_server
