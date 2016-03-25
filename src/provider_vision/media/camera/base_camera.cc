/**
 * \file	base_camera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	19/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/camera/base_camera.h"
#include <ros/ros.h>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
BaseCamera::BaseCamera(const CameraConfiguration &configuration)
    : BaseMedia(configuration.name_), config_(configuration) {
  undistord_matrix_.InitMatrices(config_.GetUndistortionMatricePath());
}

//------------------------------------------------------------------------------
//
BaseCamera::~BaseCamera() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void BaseCamera::SetFeature(const Feature &feat, float value) {
  std::stringstream ss;
  ss << std::hex << config_.guid_ << " " << GetName();
  try {
    switch (feat) {
      case Feature::SHUTTER:
        SetShutterValue(value);
        break;
      case Feature::SHUTTER_AUTO:
        SetShutterAuto();
        break;
      case Feature::SHUTTER_MANUAL:
        SetShutterManual();
        break;
      case Feature::GAIN_AUTO:
        SetGainAuto();
        break;
      case Feature::GAIN_MANUAL:
        SetGainManual();
        break;
      case Feature::GAIN:
        SetGainValue(value);
        break;
      case Feature::FRAMERATE:
        SetFrameRateValue(value);
      case Feature::WHITE_BALANCE_AUTO:
        SetWhiteBalanceAuto();
        break;
      case Feature::WHITE_BALANCE_MANUAL:
        SetWhiteBalanceManual();
        break;
      case Feature::WHITE_BALANCE_BLUE:
        SetWhiteBalanceBlueValue(value);
        break;
      case Feature::WHITE_BALANCE_RED:
        SetWhiteBalanceRedValue(value);
        break;
      case Feature::EXPOSURE:
        SetExposureValue(value);
        break;
      case Feature::GAMMA:
        SetGammaValue(value);
        break;
      case Feature::SATURATION:
        SetSaturationValue(value);
        break;
      case Feature::ERROR_FEATURE:
        break;
    }
  } catch (const std::runtime_error &e) {
    ROS_ERROR("%s", e.what());
  }
}

//------------------------------------------------------------------------------
//
float BaseCamera::GetFeature(const Feature &feat) const {
  try {
    switch (feat) {
      case Feature::SHUTTER:
        return GetShutterValue();
      case Feature::SHUTTER_AUTO:
        return GetShutterMode();
      case Feature::SHUTTER_MANUAL:
        return (static_cast<int>(GetShutterMode()) + 1) % 2;
      case Feature::FRAMERATE:
        return GetFrameRateValue();
      case Feature::WHITE_BALANCE_AUTO:
        return GetWhiteBalanceMode();
      case Feature::WHITE_BALANCE_MANUAL:
        return (static_cast<int>(GetWhiteBalanceMode()) + 1) % 2;
      case Feature::WHITE_BALANCE_BLUE:
        return GetWhiteBalanceBlue();
      case Feature::WHITE_BALANCE_RED:
        return GetWhiteBalanceRed();
      case Feature::GAIN:
        return GetGainValue();
      case Feature::GAMMA:
        return GetGammaValue();
      case Feature::EXPOSURE:
        return GetExposureValue();
      case Feature::SATURATION:
        return GetSaturationValue();
      case Feature::ERROR_FEATURE:
      default:
        return -1.0f;
    }
  } catch (const std::runtime_error &e) {
    ROS_ERROR("%s", e.what());
    return -1.0f;
  }
}

//------------------------------------------------------------------------------
//

void BaseCamera::Calibrate(){
  //Parametre a placer ds yml plus tard
  const float limitGain = 100.f;
  const float limitExposure = 100.f;
  const float msvUniform = 2.5f;

  cv::Mat img, l_hist;
  NextImage(img);

  std::vector<cv::Mat> luv_planes;
  cv::split(img,luv_planes);

  int histSize = 256;
  float range[] = {0,256};
  const float* histRange = {range};

  bool uniform = true, accumulate = false;

  cv::calcHist(&luv_planes[0],1,0,cv::Mat(),l_hist,1,&histSize,&histRange,
               uniform,accumulate);

  float msv = MSV(l_hist,5);

  if (msv != msvUniform){

  }
}

//------------------------------------------------------------------------------
//

float BaseCamera::MSV(const cv::Mat &img, int nbrRegion) {
  float num = 0.f, deno = 0.f;
  int inter = std::ceil(256/nbrRegion);
  for (int j = 0; j < nbrRegion; ++j) {
    for (int i = j*inter; i < (j+1)*inter ; ++i) {
      deno += img.at<float>(i);
      num += img.at<float>(i);
    }
    num *= (j+1);
  }
  return num/deno;
}

//------------------------------------------------------------------------------
//
double BaseCamera::UpdatePID(const std::shared_ptr<SPid> &pid, double error,
                             double position) ATLAS_NOEXCEPT {
  double pTerm, dTerm, iTerm;
  pTerm = pid->pGain * error;
  // calculate the proportional term
  // calculate the integral state with appropriate limiting
  pid->iState += error;
  if (pid->iState > pid->iMax)
    pid->iState = pid->iMax;
  else if (pid->iState < pid->iMin)
    pid->iState = pid->iMin;
  iTerm = pid->iGain * pid->iState;  // calculate the integral term
  dTerm = pid->dGain * (pid->dState - position);
  pid->dState = position;
  return pTerm + dTerm + iTerm;
}

}  // namespace provider_vision
