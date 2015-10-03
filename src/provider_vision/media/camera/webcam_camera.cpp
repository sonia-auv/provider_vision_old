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
      cv::VideoCapture(0)
    {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera(int webcamIdx) noexcept
    : BaseCamera(CameraConfiguration("Webcam")) ,
      cv::VideoCapture(webcamIdx)
      {
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
bool WebcamCamera::Start() {
  // Construction also start the camera for a videoCapture
  if ( IsOpened())
    status_ = Status::STREAMING;
  return IsOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::Stop() {
  // Always stream when asking to capture only...
  if ( IsOpened())
    status_ = Status::OPEN;
  return IsOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::NextImage(cv::Mat &image) {
  if (isOpened()) {
    this->operator>>(image);
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::Open() {
  // Already been open at constructor.
  if (!IsOpened()) {
    open(0);
  }

  return IsOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::Close() {
  if (IsOpened()) {
    release();
  }
  return !IsOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::SetFeature(const Feature &feat, float value) { return true; }

//------------------------------------------------------------------------------
//
float WebcamCamera::GetFeature(const Feature &feat) const {
  //  float val = 0.0f;
  //  if (feat == Feature::FRAMERATE)
  //  {
  //    val = static_cast<float>(this->get(CV_CAP_PROP_FPS));
  //  }

  return 0.0f;
}

}  // namespace vision_server
