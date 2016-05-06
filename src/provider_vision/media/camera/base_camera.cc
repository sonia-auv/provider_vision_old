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

  gammaPid_.dState = configuration.gamma_;
  gammaPid_.iState = configuration.gamma_iState_;
  gammaPid_.iMin = configuration.gamma_iMin_;
  gammaPid_.iMax = configuration.gamma_iMax_;
  gammaPid_.iGain = configuration.gamma_iGain_;
  gammaPid_.pGain = configuration.gamma_pGain_;
  gammaPid_.dGain = configuration.gamma_dGain_;

  gainPid_.dState = configuration.gain_;
  gainPid_.iState = configuration.gain_iState_;
  gainPid_.iMin = configuration.gain_iMin_;
  gainPid_.iMax = configuration.gain_iMax_;
  gainPid_.iGain = configuration.gain_iGain_;
  gainPid_.pGain = configuration.gain_pGain_;
  gainPid_.dGain = configuration.gain_dGain_;

  exposurePid_.dState = configuration.exposure_;
  exposurePid_.iState = configuration.exposure_iState_;
  exposurePid_.iMin = configuration.exposure_iMin_;
  exposurePid_.iMax = configuration.exposure_iMax_;
  exposurePid_.iGain = configuration.exposure_iGain_;
  exposurePid_.pGain = configuration.exposure_pGain_;
  exposurePid_.dGain = configuration.exposure_dGain_;

  saturationPid_.dState = configuration.saturation_;
  saturationPid_.iState = configuration.saturation_iState_;
  saturationPid_.iMin = configuration.saturation_iMin_;
  saturationPid_.iMax = configuration.saturation_iMax_;
  saturationPid_.iGain = configuration.saturation_iGain_;
  saturationPid_.pGain = configuration.saturation_pGain_;
  saturationPid_.dGain = configuration.saturation_dGain_;
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

void BaseCamera::Calibrate(cv::Mat const &img) {
  // Parametre a placer ds yaml plus tard
  const float limitGain = 480.f;
  const float limitExposure = 80.f;
  const float msvUniform = 2.5f;

  cv::Mat l_hist, s_hist, luvImg, hsvImg;

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
  try {
    ROS_INFO_STREAM("MSV: " << msv);
    if (msv != msvUniform) {
      if (GetExposureValue() > limitExposure && GetGainValue() > limitGain) {
        double error = fabs(GetGammaValue() - current_features_.gamma);
        current_features_.gamma = UpdatePID(gammaPid_, error, GetGammaValue());
        SetGammaValue(current_features_.gamma);
        ROS_INFO_STREAM("Gamma: " << current_features_.gamma);
      } else if (GetGainValue() > limitGain) {
        double error = fabs(GetExposureValue() - current_features_.exposure);
        current_features_.exposure =
            UpdatePID(exposurePid_, error, GetExposureValue());
        SetExposureValue(current_features_.exposure);
        ROS_INFO_STREAM("Exposure: " << current_features_.exposure);
      } else {
        double error = fabs(GetGainValue() - current_features_.gain);
        current_features_.gain = UpdatePID(gainPid_, error, GetGainValue());
        SetGainValue(current_features_.gain);
        ROS_INFO_STREAM("Gain: " << current_features_.gain);
      }
    }
    if (msv > 2 && msv < 3) {
      cv::cvtColor(img, hsvImg, CV_RGB2HSV_FULL);
      cv::split(hsvImg, hsv_planes);
      cv::calcHist(&hsv_planes[1], 1, 0, cv::Mat(), s_hist, 1, &histSize,
                   &histRange, uniform, accumulate);

      msv = MSV(s_hist, 5);
      if (msv != msvUniform) {
        double error =
            fabs(GetSaturationValue() - current_features_.saturation);
        current_features_.saturation =
            UpdatePID(saturationPid_, error, GetSaturationValue());
        SetSaturationValue(current_features_.saturation);
        ROS_INFO_STREAM("Saturation: " << current_features_.saturation);
      }
    }
  } catch (const std::exception &e) {
    ROS_ERROR("Error in calibrate camera: %s.\n", e.what());
  }
}

//------------------------------------------------------------------------------
//

float BaseCamera::MSV(const cv::Mat &img, int nbrRegion) {
  float num = 0.f, deno = 0.f;
  int inter = std::ceil(256 / nbrRegion);
  for (int j = 0; j < nbrRegion; ++j) {
    float num_buff = 0.f;
    for (int i = j * inter; i < (j + 1) * inter; ++i) {
      deno += img.at<float>(i);
      num_buff += img.at<float>(i);
    }
    num += num_buff * (j + 1);
  }
  return num / deno;
}

//------------------------------------------------------------------------------
//
double BaseCamera::UpdatePID(SPid &pid, double error,
                             double position) ATLAS_NOEXCEPT {
  double pTerm = 0, dTerm = 0, iTerm = 0;
  pTerm = pid.pGain * error;
  // calculate the proportional term
  // calculate the integral state with appropriate limiting
  pid.iState += error;
  if (pid.iState > pid.iMax)
    pid.iState = pid.iMax;
  else if (pid.iState < pid.iMin)
    pid.iState = pid.iMin;
  iTerm = pid.iGain * pid.iState;  // calculate the integral term
  dTerm = pid.dGain * fabs(pid.dState - position);
  pid.dState = position;
  return pTerm + dTerm + iTerm;
}

}  // namespace provider_vision
