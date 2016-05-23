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
void FileContext::OpenMedia(const std::string &name) {
  BaseMedia::Ptr media = GetMedia(name);
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
      throw std::invalid_argument("Not my camera type");
    }
  }
}

//------------------------------------------------------------------------------
//
void FileContext::CloseMedia(const std::string &name) {
  auto file = GetMedia(name);
  if (file != nullptr) {
    file->Close();
    EraseMedia(name);
  }
}

//------------------------------------------------------------------------------
//
void FileContext::StartStreamingMedia(const std::string &name) {
  auto file = GetMedia(name);
  // File might not be running (new video)
  if (file == nullptr) {
    OpenMedia(name);
    file = GetMedia(name);
  }
  if (file != nullptr) {
    file->StartStreaming();
  }
}

//------------------------------------------------------------------------------
//
void FileContext::StopStreamingMedia(const std::string &name) {
  auto file = GetMedia(name);
  if (file != nullptr) {
    file->StopStreaming();
  }
}

//------------------------------------------------------------------------------
//
bool FileContext::ContainsMedia(const std::string &nameMedia) const {
  bool result = false;
  if (GetMedia(nameMedia)) {
    result = true;
  } else if (GetMediaType(nameMedia) != MediaType::NONE) {
    result = true;
  }
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
void FileContext::Run() {}

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

}  // namespace provider_vision
