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
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera()
    : cv::VideoCapture(0), BaseCamera(CameraConfiguration("Webcam")) {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera(int webcamIdx)
    : cv::VideoCapture(webcamIdx),
      // three because...
      BaseCamera(CameraConfiguration("Webcam")) {
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
  if (isOpened()) status_ = Status::STREAMING;
  return isOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::Stop() {
  // Always stream when asking to capture only...
  if (isOpened()) status_ = Status::OPEN;
  return isOpened();
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
  if (!isOpened()) {
    open(0);
  }

  return isOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::Close() {
  if (isOpened()) {
    release();
  }
  return !isOpened();
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::SetFeature(const Feature &feat, float value) { return true; }

//------------------------------------------------------------------------------
//
float WebcamCamera::GetFeature(const Feature &feat) const{
//  float val = 0.0f;
//  if (feat == Feature::FRAMERATE)
//  {
//    val = static_cast<float>(this->get(CV_CAP_PROP_FPS));
//  }

  return 0.0f;
}

}  // namespace vision_server
