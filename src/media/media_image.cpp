/**
 * \file	MMImage.cpp
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
#include "media/media_image.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
MMImage::MMImage(std::string path_to_file)
    : _path(path_to_file),
      // Two because...
      Media(CameraID(path_to_file, 1)) {
  LoadImage(_path);
}

//------------------------------------------------------------------------------
//
MMImage::MMImage() : _path(""), Media(CameraID("NO_PATH", 2)) {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool MMImage::LoadImage(std::string path_to_file) {
  _image =
      cv::imread(path_to_file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  if (_image.empty()) {
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
inline std::vector<std::string> MMImage::getCommands() const {
  // its an image, no command.
  return std::vector<std::string>();
}

//------------------------------------------------------------------------------
//
bool MMImage::Start() {
  // if the media is open, then the image is not empty.
  if (_image.empty()) {
    return _image.empty();
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool MMImage::Stop() {
  // At destruction, will close the image... No method to close
  return true;
}

//------------------------------------------------------------------------------
//
bool MMImage::NextImage(cv::Mat &image) {
  // Here, since cv::Mat are smart pointer, we can just
  // clone the image, and the "garbage collection"
  // will be handle later on in the program.
  if (!_image.empty()) {
    image = _image.clone();
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
std::string MMImage::GetName() const { return _path; }

}  // namespace vision_server
