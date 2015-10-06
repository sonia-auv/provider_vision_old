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
ImageFile::ImageFile(std::string path_to_file)
    : BaseMedia(CameraConfiguration(path_to_file)), path_(path_to_file) {
  LoadImage(path_);
}

//------------------------------------------------------------------------------
//
ImageFile::ImageFile() : BaseMedia(CameraConfiguration("NO_PATH")), path_("") {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool ImageFile::LoadImage(std::string path_to_file) {
  image_ =
      cv::imread(path_to_file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  if (image_.empty()) {
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool ImageFile::Start() {
  // if the media is open, then the image is not empty.
  if (image_.empty()) {
    return image_.empty();
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool ImageFile::Stop() {
  // At destruction, will close the image... No method to close
  return true;
}

//------------------------------------------------------------------------------
//
bool ImageFile::NextImage(cv::Mat &image) {
  // Here, since cv::Mat are smart pointer, we can just
  // clone the image, and the "garbage collection"
  // will be handle later on in the program.
  if (!image_.empty()) {
    image = image_.clone();
    return true;
  }
  return false;
}
}  // namespace vision_server
