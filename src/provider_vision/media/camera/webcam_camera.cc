/**
 * \file	webcam_camera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/camera/webcam_camera.h"

namespace provider_vision {

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
float WebcamCamera::GetGainValue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetGammaValue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetExposureValue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetSaturationValue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetGainAuto() {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetGainManual() {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetGainValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetGammaValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetExposureValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetSaturationValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetShutterValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetShutterAuto() {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetShutterManual() {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetShutterMode() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetShutterValue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetFrameRateValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetFrameRateValue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetWhiteBalanceAuto() {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetWhiteBalanceMan() {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetWhiteBalanceMode() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetWhiteBalanceRedValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
void WebcamCamera::SetWhiteBalanceBlueValue(float value) {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetWhiteBalanceRed() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

//------------------------------------------------------------------------------
//
float WebcamCamera::GetWhiteBalanceBlue() const {
  throw std::runtime_error("This feature is not supported with this camera");
}

}  // namespace provider_vision
