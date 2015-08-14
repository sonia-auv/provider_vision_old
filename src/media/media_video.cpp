/**
 * \file	MMVideo.cpp
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
#include "media/media_video.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
MMVideo::MMVideo(std::string path_to_file, bool looping)
    : VideoCapture(path_to_file),
      // Two because...
      Media(CameraID(path_to_file, 2)),
      _path(path_to_file),
      _currentImage(),
      _looping(looping) {
  LoadVideo(_path);
}

//------------------------------------------------------------------------------
//
MMVideo::MMVideo()
    : _path(std::string("")),
      Media(CameraID("NO_PATH", 2)),
      _currentImage(),
      _looping(true) {}

//------------------------------------------------------------------------------
//
MMVideo::~MMVideo() {
  if (isOpened()) {
    release();
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
inline std::vector<std::string> MMVideo::getCommands() const {
  // Should implement play, pause, stop, forward, backward, etc.
  return std::vector<std::string>();
}

//------------------------------------------------------------------------------
//
void MMVideo::SetPathToVideo(std::string full_path) { _path = full_path; }

//------------------------------------------------------------------------------
//
void MMVideo::SetLooping(bool looping) { _looping = looping; }

//------------------------------------------------------------------------------
//
bool MMVideo::LoadVideo(std::string path_to_file) {
  return this->open(path_to_file);
}

//------------------------------------------------------------------------------
//
bool MMVideo::Start() {
  if (!isOpened()) {
    LoadVideo(_path);
  }
  return isOpened();
}

//------------------------------------------------------------------------------
//
bool MMVideo::Stop() {
  if (isOpened()) release();
  return true;
}

//------------------------------------------------------------------------------
//
bool MMVideo::NextImage(cv::Mat &image) {
  // Here, since cv::Mat are smart pointer, we can just
  // clone the image, and the "garbage collection"
  // will be handle later on in the program.
  bool video_status = false;
  if (isOpened()) {
    // Clear the previous image.
    _currentImage = cv::Mat();

    this->operator>>(_currentImage);
    if (!_currentImage.empty()) {
      // Next image has loaded
      image = _currentImage;
      video_status = true;
    } else if (_looping) {
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

//------------------------------------------------------------------------------
//
inline std::string MMVideo::GetName() const { return _path; }

}  // namespace vision_server
