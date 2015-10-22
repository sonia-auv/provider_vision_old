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
      cv::VideoCapture() {
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
void WebcamCamera::Open() {
  // Already been open at constructor.
  if (!IsOpened()) {
    open(0);
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::Close() {
  if (IsOpened()) {
    release();
    status_ = Status::CLOSE;
  }
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetStreamingModeOn() { status_ = Status::STREAMING; }

//------------------------------------------------------------------------------
//
void WebcamCamera::SetStreamingModeOff() { status_ = Status::OPEN; }

//------------------------------------------------------------------------------
//
void WebcamCamera::NextImage(cv::Mat &image) {
  if (isOpened() && (status_ == Status::OPEN || status_ == Status::STREAMING)) {
    operator>>(image);
  } else {
    image = cv::Mat().clone();
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
float WebcamCamera::GetFeature(const Feature &feat) const { return 0.0f; }

}  // namespace vision_server
