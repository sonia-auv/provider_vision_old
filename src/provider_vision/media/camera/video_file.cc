/**
 * \file	video_file.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <string>
#include <vector>
#include "provider_vision/media/camera/video_file.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
VideoFile::VideoFile(const std::string &path_to_file, bool looping)
    : BaseMedia(CameraConfiguration(path_to_file)),
      VideoCapture(path_to_file),
      current_image_(),
      path_(path_to_file),
      looping_(looping) {
  LoadVideo(path_);
}

//------------------------------------------------------------------------------
//
VideoFile::VideoFile()
    : BaseMedia(CameraConfiguration("NO_PATH")),
      current_image_(),
      path_(std::string("")),
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
void VideoFile::Open() {
  if (isOpened()) {
    throw std::logic_error("The video is already opened.");
  } else {
    LoadVideo(path_);
  }

  if (!isOpened()) {
    // Check if the video could be opened.
    throw std::runtime_error("The video could not be opened.");
  }
  status_ = Status::OPEN;
}

//------------------------------------------------------------------------------
//
void VideoFile::Close() {
  if (!isOpened()) {
    throw std::logic_error("The video is not opened.");
  } else {
    release();
  }

  if (isOpened()) {
    throw std::runtime_error("The video could not be closed.");
  }
  status_ = Status::CLOSE;
}

//------------------------------------------------------------------------------
//
void VideoFile::SetStreamingModeOn() { status_ = Status::STREAMING; }

//------------------------------------------------------------------------------
//
void VideoFile::SetStreamingModeOff() { status_ = Status::OPEN; }

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
  return open(path_to_file);
}

//------------------------------------------------------------------------------
//
void VideoFile::NextImage(cv::Mat &image) {
  if (isOpened()) {
    // Clear the previous image.
    current_image_ = cv::Mat();

    operator>>(current_image_);
    if (!current_image_.empty()) {
      // Next image has loaded
      image = current_image_;
    } else if (looping_) {
      // end of sequence, going back to first frame.
      // Delay will happen, but it's part of the lib...
      set(CV_CAP_PROP_POS_AVI_RATIO, 0);
    } else {
      // No more frame and not looping, end of sequence.
      throw std::logic_error("No image could be acquiered from this media");
    }
  }
}

}  // namespace provider_vision
