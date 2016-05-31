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

#include <provider_vision/media/context/file_context.h>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N
const char *FileContext::DRIVER_TAG = "File context";
//------------------------------------------------------------------------------
//
FileContext::FileContext() noexcept : BaseContext() {}

//------------------------------------------------------------------------------
//
FileContext::~FileContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void FileContext::CloseContext() { media_list_.clear(); }

//------------------------------------------------------------------------------
//
bool FileContext::OpenMedia(const std::string &name) {
  BaseMedia::Ptr media = GetMedia(name);

  // If we did not find the media, this means it is not open, so we need to
  // open it.
  if (!media) {
    MediaType type = GetMediaType(name);

    if (type == MediaType::IMAGE) {
      ImageFile::Ptr file(std::make_shared<ImageFile>(name));
      file->Open();
      media_list_.push_back(std::dynamic_pointer_cast<BaseMedia>(file));
    } else if (type == MediaType::VIDEO) {
      VideoFile::Ptr file(std::make_shared<VideoFile>(name));
      media_list_.push_back(std::dynamic_pointer_cast<BaseMedia>(file));
    } else {
      ROS_ERROR("%s Not my media type", DRIVER_TAG);
      return false;
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool FileContext::CloseMedia(const std::string &name) {
  auto file = GetMedia(name);
  if (file) {
    // Here, we do not look for success, erase media will delete it anyway.
    file->Close();
    EraseMedia(name);
    return true;
  }
  ROS_ERROR("%s Media not found", DRIVER_TAG);
  return false;
}

//------------------------------------------------------------------------------
//
bool FileContext::StartStreamingMedia(const std::string &name) {
  auto file = GetMedia(name);
  // File might not be running (new video)
  if (file) {
    return file->StartStreaming();
  } else {
    return OpenMedia(name);
  }
}

//------------------------------------------------------------------------------
//
bool FileContext::StopStreamingMedia(const std::string &name) {
  auto file = GetMedia(name);
  if (file) {
    return file->StopStreaming();
  }
  ROS_ERROR("%s Media not found", DRIVER_TAG);
  return false;
}

//------------------------------------------------------------------------------
//
bool FileContext::ContainsMedia(const std::string &nameMedia) const {
  bool result = false;
  // This function might be called when creating a video/image file, so we
  // won't have in our media BUT it is still ours.
  if (GetMedia(nameMedia)) {
    result = true;
  } else if (GetMediaType(nameMedia) != MediaType::NONE) {
    result = true;
  }
  return result;
}

//------------------------------------------------------------------------------
//
bool FileContext::SetFeature(const BaseCamera::Feature &feat,
                             const std::string &name, boost::any &val) {
  // Here we make a choice to return false, since if you try to set a parameter
  // on a file, there is something wrong...
  return false;
}

//------------------------------------------------------------------------------
//
bool FileContext::GetFeature(const BaseCamera::Feature &feat,
                             const std::string &name, boost::any &val) const {
  // Here we make a choice to return false, since if you try to set a parameter
  // on a file, there is something wrong...
  return false;
}

//------------------------------------------------------------------------------
//
void FileContext::Run() {}

//------------------------------------------------------------------------------
//
bool FileContext::WatchDogFunc() { return true; }

//------------------------------------------------------------------------------
//
FileContext::MediaType FileContext::GetMediaType(const std::string &nameMedia)
    const {
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

}  // namespace provider_vision
