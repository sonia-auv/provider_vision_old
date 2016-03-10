/**
 * \file	image_file.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/camera/image_file.h"
#include <string>
#include <vector>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ImageFile::ImageFile(const std::string &path_to_file) noexcept
    : BaseMedia(path_to_file),
      path_(path_to_file) {}

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
