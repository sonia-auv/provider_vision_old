/**
 * \file	WebcamCamera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "provider_vision/media/camera/webcam_camera.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera() noexcept
    : BaseCamera(CameraConfiguration("Webcam")),
      cv::VideoCapture(0) {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera(int webcamIdx) noexcept
    : BaseCamera(CameraConfiguration("Webcam")),
      cv::VideoCapture(webcamIdx) {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
WebcamCamera::~WebcamCamera() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void WebcamCamera::Start() {
  if (isOpened()) {
    throw std::logic_error("The video is already opened.");
  } else {
    status_ = Status::STREAMING;
  }

  if (!isOpened()) {
    // Check if the video could be opened.
    throw std::runtime_error("The video could not be opened.");
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::Stop() {
  if (!isOpened()) {
    throw std::logic_error("The video is not opened.");
  } else {
    release();
    status_ = Status::CLOSE;
  }

  if (isOpened()) {
    // Check if the video could be opened.
    throw std::runtime_error("The video could not be closed.");
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::NextImage(cv::Mat &image) {
  if (isOpened()) {
    operator>>(image);
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::Open() {
  // Already been open at constructor.
  if (!IsOpened()) {
    open(0);
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::Close() {
  if (IsOpened()) {
    release();
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetFeature(const Feature &feat, float value) {
  // For now we do not handle setting a feature on the webcam camera, it might
  // come later.
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetFeature(const Feature &feat) const {
  return 0.0f;
}

}  // namespace vision_server
