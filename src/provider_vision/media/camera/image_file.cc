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

#include "provider_vision/media/camera/image_file.h"
#include <string>
#include <vector>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ImageFile::ImageFile(const std::string &path_to_file) noexcept
    : BaseMedia(path_to_file), path_(path_to_file) {}

//------------------------------------------------------------------------------
//
ImageFile::~ImageFile() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageFile::Open() {
  image_ = cv::imread(path_, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  if (image_.empty()) {
    throw std::runtime_error("There is no image file with this path");
  }
  status_ = Status::OPEN;
}

//------------------------------------------------------------------------------
//
void ImageFile::Close() { status_ = Status::CLOSE; }

//------------------------------------------------------------------------------
//
void ImageFile::SetStreamingModeOn() { status_ = Status::STREAMING; }

//------------------------------------------------------------------------------
//
void ImageFile::SetStreamingModeOff() { status_ = Status::OPEN; }

//------------------------------------------------------------------------------
//
void ImageFile::NextImage(cv::Mat &image) {
  if (!image_.empty()) {
    if (IsClosed()) {
      image = cv::Mat().clone();
    } else {
      image_.copyTo(image);
    }
  } else {
    throw std::runtime_error(
        "The image could not be loaded, an error occurenced");
  }
}

//------------------------------------------------------------------------------
//
void ImageFile::NextImageCopy(cv::Mat &image) noexcept { NextImage(image); }

}  // namespace provider_vision
