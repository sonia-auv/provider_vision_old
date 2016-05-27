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

#include "provider_vision/server/media_manager.h"
#include <lib_atlas/macros.h>
#include <stdexcept>
#include <vector>
#include "provider_vision/config.h"
#include "provider_vision/media/context/dc1394_context.h"
#ifndef OS_DARWIN
#include "provider_vision/media/context/gige_context.h"
#endif  // OS_DARWIN
#include <ros/console.h>
#include "provider_vision/media/context/file_context.h"
#include "provider_vision/media/context/webcam_context.h"

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
  std::vector<std::string> camera_names_dc1394;
  nh_.getParam("/provider_vision/active_dc1394", camera_names_dc1394);
  if (!camera_names_dc1394.empty()) {
    std::vector<CameraConfiguration> configurations;
    for (const auto &camera_dc : camera_names_dc1394) {
      configurations.push_back(CameraConfiguration(nh_, camera_dc));
    }
    contexts_.push_back(std::make_shared<DC1394Context>(configurations));
  }

#ifndef OS_DARWIN
  // Creating the GigE context
  std::vector<std::string> camera_names_gige;
  nh_.getParam("/provider_vision/active_gige", camera_names_gige);
  if (!camera_names_gige.empty()) {
    std::vector<CameraConfiguration> configurations;
    for (const auto &camera_gige : camera_names_gige) {
      configurations.push_back(CameraConfiguration(nh_, camera_gige));
    }
    contexts_.push_back(std::make_shared<GigeContext>(configurations));
  }
#endif

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
  if (context) {
    context->OpenMedia(media_name);
  } else {
    throw std::invalid_argument("The media is not part of a context.");
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::CloseMedia(const std::string &media_name) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context) {
    context->CloseMedia(media_name);
  } else {
    throw std::invalid_argument("The media is not part of a context.");
  }
}

//------------------------------------------------------------------------------
//
MediaStreamer::Ptr MediaManager::StartStreamingMedia(
    const std::string &media_name) {
  MediaStreamer::Ptr streamer(nullptr);

  // If the media is already streaming, return the streamer
  if (IsMediaStreaming(media_name)) {
    streamer = GetMediaStreamer(media_name);
  } else {
    // Find which context to owns the media.
    BaseContext::Ptr context = GetContextFromMedia(media_name);
    if (!context) {
      throw std::invalid_argument("The media is not part of a context.");
    }
    // The context set the media to stream
    context->StartStreamingMedia(media_name);

    // Get the media to create a streamer
    BaseMedia::Ptr media = context->GetMedia(media_name);
    if (media) {
      // if the media is valid, create the streamer
      streamer = std::make_shared<MediaStreamer>(media, 30);
      if (streamer) {
        streamer->StartStreaming();
        // Keep it in the list of Streaming media
        AddMediaStreamer(streamer);
      } else {
        throw std::runtime_error("Streamer failed to be created");
      }
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
  if (streamer) {
    StopStreamingMedia(streamer);
    RemoveMediaStreamer(media);
    ROS_INFO("Media is stopped.");
  } else {
    throw std::invalid_argument("Streamer could not be found");
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::StopStreamingMedia(
    const MediaStreamer::Ptr &streamer) noexcept {
  if (streamer->ObserverCount() > 1) {
    ROS_INFO(
        "Not stopping the media because at least another observer is using "
        "it.");
  } else {
    if (streamer->IsStreaming()) {
      streamer->StopStreaming();
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
                                    const std::string &feature,
                                    boost::any &value) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context) {
    context->SetFeature(GetFeatureFromName(feature), media_name, value);
  } else {
    throw std::invalid_argument("Context not found for this media");
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::GetCameraFeature(const std::string &media_name,
                                    const std::string &feature,
                                    boost::any &value) const {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context) {
    context->GetFeature(GetFeatureFromName(feature), media_name, value);
  } else {
    throw std::invalid_argument("Context not found for this media");
  }
}

//------------------------------------------------------------------------------
//
BaseContext::Ptr MediaManager::GetContextFromMedia(const std::string &name)
    const {
  BaseContext::Ptr context_ptr(nullptr);
  for (auto &context : contexts_) {
    if (context->ContainsMedia(name)) {
      context_ptr = context;
    }
  }
  return context_ptr;
}

//------------------------------------------------------------------------------
//
BaseCamera::Feature MediaManager::GetFeatureFromName(const std::string &name)
    const {
  if (name == "SHUTTER_AUTO") {
    return BaseCamera::Feature::SHUTTER_MODE;
  } else if (name == "SHUTTER") {
    return BaseCamera::Feature::SHUTTER_VALUE;
  } else if (name == "WHITE_BALANCE_AUTO") {
    return BaseCamera::Feature::WHITE_BALANCE_MODE;
  } else if (name == "WHITE_BALANCE_RED") {
    return BaseCamera::Feature::WHITE_BALANCE_RED_VALUE;
  } else if (name == "WHITE_BALANCE_BLUE") {
    return BaseCamera::Feature::WHITE_BALANCE_BLUE_VALUE;
  } else if (name == "FRAMERATE") {
    return BaseCamera::Feature::FRAMERATE_VALUE;
  } else {
    throw std::invalid_argument("No feature with this name");
  }
}

}  // namespace provider_vision
