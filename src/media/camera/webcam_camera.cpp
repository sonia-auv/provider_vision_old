/**
 * \file	CAMWebcam.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "media/camera/webcam_camera.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMWebcam::CAMWebcam()
: cv::VideoCapture(0),
  BaseCamera(CameraConfiguration("Webcam")) {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
CAMWebcam::CAMWebcam(int webcamIdx)
    : cv::VideoCapture(webcamIdx),
      // three because...
      BaseCamera(CameraConfiguration("Webcam")) {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
CAMWebcam::~CAMWebcam() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool CAMWebcam::Start() {
  // Construction also start the camera for a videoCapture
  if (isOpened()) status_ = Status::STREAMING;
  return isOpened();
}

//------------------------------------------------------------------------------
//
bool CAMWebcam::Stop() {
  // Always stream when asking to capture only...
  if (isOpened()) status_ = Status::OPEN;
  return isOpened();
}

//------------------------------------------------------------------------------
//
bool CAMWebcam::NextImage(cv::Mat &image) {
  if (isOpened()) {
    this->operator>>(image);
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool CAMWebcam::Open() {
  // Already been open at constructor.
  if (!isOpened()) {
    open(0);
  }

  return isOpened();
}

//------------------------------------------------------------------------------
//
bool CAMWebcam::Close() {
  if (isOpened()) {
    release();
  }
  return !isOpened();
}

//------------------------------------------------------------------------------
//
bool CAMWebcam::SetFeature(const Feature &feat, float value) { return true; }

//------------------------------------------------------------------------------
//
float CAMWebcam::GetFeature(const Feature &feat) {
  float val = 0.f;
  if (feat == Feature::FRAMERATE) {
    val = static_cast<float>(get(CV_CAP_PROP_FPS));
  }

  return val;
}

}  // namespace vision_server
