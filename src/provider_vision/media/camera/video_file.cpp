/**
 * \file	VideoFile.cpp
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
#include "provider_vision/media/camera/video_file.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
VideoFile::VideoFile(const std::string &path_to_file, bool looping)
    : VideoCapture(path_to_file),
      BaseMedia(CameraConfiguration(path_to_file)),
      path_(path_to_file),
      current_image_(),
      looping_(looping) {
  LoadVideo(path_);
}

//------------------------------------------------------------------------------
//
VideoFile::VideoFile()
    : path_(std::string("")),
      BaseMedia(CameraConfiguration("NO_PATH")),
      current_image_(),
      looping_(true) {}

//------------------------------------------------------------------------------
//
VideoFile::~VideoFile() {
  if (isOpened()) {
    release();
  }
}

//==============================================================================
// M E T H O D   S E C T I O N
//------------------------------------------------------------------------------
//
void VideoFile::SetPathToVideo(const std::string &full_path) {
  path_ = full_path;
}

//------------------------------------------------------------------------------
//
void VideoFile::SetLooping(bool looping) { looping_ = looping; }

//------------------------------------------------------------------------------
//
bool VideoFile::LoadVideo(const std::string &path_to_file) {
  return this->open(path_to_file);
}

//------------------------------------------------------------------------------
//
bool VideoFile::Start() {
  if (!isOpened()) {
    LoadVideo(path_);
  }
  return isOpened();
}

//------------------------------------------------------------------------------
//
bool VideoFile::Stop() {
  if (isOpened()) release();
  return true;
}

//------------------------------------------------------------------------------
//
bool VideoFile::NextImage(cv::Mat &image) {
  // Here, since cv::Mat are smart pointer, we can just
  // clone the image, and the "garbage collection"
  // will be handle later on in the program.
  bool video_status = false;
  if (isOpened()) {
    // Clear the previous image.
    current_image_ = cv::Mat();

    this->operator>>(current_image_);
    if (!current_image_.empty()) {
      // Next image has loaded
      image = current_image_;
      video_status = true;
    } else if (looping_) {
      // end of sequence, going back to first frame.
      // Delay will happen, but it's part of the lib...
      set(CV_CAP_PROP_POS_AVI_RATIO, 0);
      video_status = true;
    } else {
      // No more frame and not looping, end of sequence.
      video_status = false;
    }
  }

  return video_status;
}
}  // namespace vision_server
