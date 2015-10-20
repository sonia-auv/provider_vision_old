/**
 * \file	ImageFile.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <vector>
#include "provider_vision/media/camera/image_file.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ImageFile::ImageFile(const std::string &path_to_file) noexcept
    : BaseMedia(CameraConfiguration(path_to_file)),
      path_(path_to_file) {}

//------------------------------------------------------------------------------
//
ImageFile::~ImageFile() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageFile::Start() {
  status_ = Status::STREAMING;
  image_ = cv::imread(path_, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  if (image_.empty()) {
    throw std::runtime_error("There is no image file with this path");
  }
}

//------------------------------------------------------------------------------
//
void ImageFile::Stop() { status_ = Status::CLOSE; }

//------------------------------------------------------------------------------
//
void ImageFile::NextImage(cv::Mat &image) {
  if (!image_.empty()) {
    image_.copyTo(image);
  } else {
    throw std::runtime_error(
        "The image could not be loaded, an error occurenced");
  }
}

//------------------------------------------------------------------------------
//
void ImageFile::NextImageCopy(cv::Mat &image) noexcept { NextImage(image); }

}  // namespace vision_server
