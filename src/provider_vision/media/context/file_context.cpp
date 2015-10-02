/**
 * \file	VideoFileContext.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <provider_vision/media/context/file_context.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "provider_vision/media/camera/image_file.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
FileContext::FileContext() : BaseContext(), DRIVER_TAG("[MEDIA Driver]") {}

//------------------------------------------------------------------------------
//
FileContext::~FileContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void FileContext::InitContext(
    const std::vector<CameraConfiguration> &cam_configuration_lists) {}

//------------------------------------------------------------------------------
//
void FileContext::CloseContext() { media_list_.clear(); }

//------------------------------------------------------------------------------
//
bool FileContext::StartCamera(const std::string &name) {
  // in the videoFile context, camera in list are existing camera.
  if (GetMedia(name) != media_list_.end())
    // already existing
    return true;

  MediaType type = GetMediaType(name);
  if (type == MediaType::IMAGE) {
    ImageFile::Ptr file(new ImageFile(name));
    file->Start();
    media_list_.insert(std::make_pair(name, file));
  } else if (type == MediaType::VIDEO) {
    VideoFile::Ptr file(new VideoFile(name));
    file->Start();
    media_list_.insert(std::make_pair(name, file));
  } else {
    throw std::invalid_argument("Not my camera type");
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool FileContext::StopCamera(const std::string &name) {
  // in the videoFile context, camera in list are existing camera.
  auto file = GetMedia(name);
  if (file == media_list_.end())
    // does not exist
    return true;

  bool result = (*file).second->Stop();
  media_list_.erase(file);

  return result;
}

//------------------------------------------------------------------------------
//
bool FileContext::ContainsMedia(const std::string &nameMedia) const {
  bool result = false;
  // cherche si la camera existe
  if (GetMedia(nameMedia) != media_list_.end()) {
    // already existing
    result = true;
  }  // N'existe pas dans le systeme, est-ce qu'on peut la creer?
  else if (GetMediaType(nameMedia) != MediaType::NONE) {
    // C'est un video ou une image
    result = true;
  }

  // N'est pas un video ni une image ou un media existant pour ce driver.
  return result;
}
//------------------------------------------------------------------------------
//
void FileContext::SetFeature(BaseCamera::Feature feat, const std::string &name,
                             float val) {}

//------------------------------------------------------------------------------
//
void FileContext::GetFeature(BaseCamera::Feature feat, const std::string &name,
                             float &val) const {}

//------------------------------------------------------------------------------
//
void FileContext::run() {}

//------------------------------------------------------------------------------
//
bool FileContext::WatchDogFunc() { return true; }

//------------------------------------------------------------------------------
//
FileContext::MediaType FileContext::GetMediaType(
    const std::string &nameMedia) const {
  // on commence par rechercher une image
  if (nameMedia.find(".jpg") != std::string::npos ||
      nameMedia.find(".png") != std::string::npos ||
      nameMedia.find(".bmp") != std::string::npos) {
    return MediaType::IMAGE;
  }

  // On n'a pas trouver d'image, on recherche une vidéo
  if (nameMedia.find(".avi") != std::string::npos ||
      nameMedia.find(".mp4") != std::string::npos) {
    return MediaType::VIDEO;
  }

  return MediaType::NONE;
}

}  // namespace vision_server
