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

  current_features_.exposure = configuration.exposure_;
  current_features_.gain = configuration.gain_;
  current_features_.gamma = configuration.gamma_;
  current_features_.saturation = configuration.saturation_;

  gammaPid_->dState = configuration.gamma_;
  gammaPid_->iState = 0.0;
  gammaPid_->iMin = 1.0;
  gammaPid_->iMax = 2000;
  gammaPid_->iGain = 0.01;
  gammaPid_->pGain = 2;
  gammaPid_->dGain = 25;

  gainPid_->dState = configuration.gain_;
  gainPid_->iState = 0.0;
  gainPid_->iMin = 1.0;
  gainPid_->iMax = 2000;
  gainPid_->iGain = 0.01;
  gainPid_->pGain = 2;
  gainPid_->dGain = 25;

  exposurePid_->dState = configuration.exposure_;
  exposurePid_->iState = 0.0;
  exposurePid_->iMin = 1.0;
  exposurePid_->iMax = 2000;
  exposurePid_->iGain = 0.01;
  exposurePid_->pGain = 2;
  exposurePid_->dGain = 25;

  saturationPid_->dState = configuration.saturation_;
  saturationPid_->iState = 0.0;
  saturationPid_->iMin = 1.0;
  saturationPid_->iMax = 2000;
  saturationPid_->iGain = 0.01;
  saturationPid_->pGain = 2;
  saturationPid_->dGain = 25;
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

void BaseCamera::Calibrate() {
  // Parametre a placer ds yaml plus tard
  const float limitGain = 100.f;
  const float limitExposure = 100.f;
  const float msvUniform = 2.5f;

  cv::Mat img, l_hist, s_hist, luvImg, hsvImg;
  NextImage(img);

  cv::cvtColor(img, luvImg, CV_RGB2Luv);

  std::vector<cv::Mat> luv_planes, hsv_planes;
  cv::split(luvImg, luv_planes);

  int histSize = 256;
  float range[] = {0, 256};
  const float *histRange = {range};

  bool uniform = true, accumulate = false;

  cv::calcHist(&luv_planes[0], 1, 0, cv::Mat(), l_hist, 1, &histSize,
               &histRange, uniform, accumulate);

  float msv = MSV(l_hist, 5);

  if (msv != msvUniform) {
    if (GetExposureValue() > limitExposure && GetGainValue() > limitGain) {
      double error = fabs(GetGammaValue() - current_features_.gamma);
      current_features_.gamma = UpdatePID(gammaPid_, error, GetGammaValue());
      SetGammaValue(current_features_.gamma);
    } else if (GetGainValue() > limitGain) {
      double error = fabs(GetExposureValue() - current_features_.exposure);
      current_features_.exposure =
          UpdatePID(exposurePid_, error, GetExposureValue());
      SetExposureValue(current_features_.exposure);
    } else {
      double error = fabs(GetGainValue() - current_features_.gain);
      current_features_.gain = UpdatePID(gainPid_, error, GetGainValue());
      SetGainValue(current_features_.gain);
    }
  }
  if (msv > 2 && msv < 3) {
    cv::cvtColor(img, hsvImg, CV_RGB2HSV_FULL);
    cv::split(hsvImg, hsv_planes);
    cv::calcHist(&hsv_planes[1], 1, 0, cv::Mat(), s_hist, 1, &histSize,
                 &histRange, uniform, accumulate);

    msv = MSV(s_hist, 5);
    if (msv != msvUniform) {
      double error = fabs(GetSaturationValue() - current_features_.saturation);
      current_features_.saturation =
          UpdatePID(saturationPid_, error, GetSaturationValue());
      SetSaturationValue(current_features_.saturation);
    }
  }
}

//------------------------------------------------------------------------------
//

float BaseCamera::MSV(const cv::Mat &img, int nbrRegion) {
  float num = 0.f, deno = 0.f;
  int inter = std::ceil(256 / nbrRegion);
  for (int j = 0; j < nbrRegion; ++j) {
    for (int i = j * inter; i < (j + 1) * inter; ++i) {
      deno += img.at<float>(i);
      num += img.at<float>(i);
    }
    num *= (j + 1);
  }
  return num / deno;
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
