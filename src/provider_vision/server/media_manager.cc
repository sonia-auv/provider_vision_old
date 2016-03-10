/**
 * \file	media_manager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/server/media_manager.h"
#include <stdexcept>
#include <vector>
#include "provider_vision/media/context/dc1394_context.h"
#include "provider_vision/media/context/file_context.h"
#include "provider_vision/media/context/webcam_context.h"
#include "provider_vision/utils/config.h"
#include "ros/console.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaManager::MediaManager(const ros::NodeHandle &nh) noexcept : contexts_() {
  // Creating the Webcam context
  auto active_webcam = false;
  nh_.getParam("/provider_vision/active_webcam", active_webcam);
  if (active_webcam) {
    contexts_.push_back(std::make_shared<WebcamContext>());
  }

  // Creating the DC1394 context
  std::vector<std::string> camera_names = {"front_guppy", "bottom_guppy"};
  nh_.getParam("/provider_vision/active_dc1394", camera_names);
  if (!camera_names.empty()) {
    std::vector<CameraConfiguration> configurations;
    for (const auto &camera : camera_names) {
      configurations.push_back(CameraConfiguration(nh_, camera));
    }
    contexts_.push_back(std::make_shared<DC1394Context>(configurations));
  }

  contexts_.push_back(std::make_shared<FileContext>());
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
void MediaManager::OpenMedia(const std::string &media_name) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context == nullptr) {
    throw std::invalid_argument("The media is not part of a context.");
  }
  context->OpenMedia(media_name);
}

//------------------------------------------------------------------------------
//
void MediaManager::CloseMedia(const std::string &media_name) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context == nullptr) {
    throw std::invalid_argument("The media is not part of a context.");
  }
  context->CloseMedia(media_name);
}

//------------------------------------------------------------------------------
//
MediaStreamer::Ptr MediaManager::StartStreamingMedia(
    const std::string &media_name) {
  MediaStreamer::Ptr streamer(nullptr);

  if (IsMediaStreaming(media_name)) {
    streamer = GetMediaStreamer(media_name);
  } else {
    BaseContext::Ptr context = GetContextFromMedia(media_name);
    if (!context) {
      throw std::invalid_argument("The media is not part of a context.");
    }
    context->StartStreamingMedia(media_name);

    BaseMedia::Ptr media = context->GetMedia(media_name);
    if (media) {
      streamer = std::make_shared<MediaStreamer>(media, 30);
      if (streamer != nullptr) {
        streamer->StartStreaming();
      }
      AddMediaStreamer(streamer);
    } else {
      throw std::runtime_error("Camera failed to be created");
    }
  }
  ROS_INFO("Media is ready.");
  return streamer;
}

//------------------------------------------------------------------------------
//
void MediaManager::StopStreamingMedia(const std::string &media) noexcept {
  MediaStreamer::Ptr streamer = GetMediaStreamer(media);
  StopStreamingMedia(streamer);
  RemoveMediaStreamer(media);
  ROS_INFO("Media is stopped.");
}

//------------------------------------------------------------------------------
//
void MediaManager::StopStreamingMedia(
    const MediaStreamer::Ptr &streamer) noexcept {
  if (streamer == nullptr) {
    ROS_ERROR("Trying to stop streaming media but is null");
    return;
  }
  if (streamer->ObserverCount() > 1) {
    ROS_INFO(
        "Not stopping the media because at least another observer is using "
        "it.");
    return;
  }
  if (streamer->IsStreaming()) {
    streamer->StopStreaming();
  }
}

//------------------------------------------------------------------------------
//
const BaseMedia::Status &MediaManager::GetMediaStatus(const std::string &name) {
  auto media = GetMedia(name);
  if (media != nullptr) {
    return media->GetStatus();
  } else {
    throw std::invalid_argument("This media does not exist.");
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
size_t MediaManager::GetAllMediasCount() const noexcept {
  size_t size = 0;
  for (auto &context : contexts_) {
    size += context->GetMediaList().size();
  }
  return size;
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

}  // namespace provider_vision
