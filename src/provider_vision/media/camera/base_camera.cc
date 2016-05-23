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
#include <sonia_msgs/CameraFeatures.h>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
BaseCamera::BaseCamera(const CameraConfiguration &configuration)
    : BaseMedia(configuration.name_),
      CameraConfiguration(configuration), feature_pub_() {
  undistord_matrix_.InitMatrices(undistortion_matrice_path_);

  current_features_.exposure = configuration.exposure_;
  current_features_.gain = configuration.gain_;
  current_features_.gamma = configuration.gamma_;
  current_features_.saturation = configuration.saturation_;

  gamma_pid_.d_state = configuration.gamma_;
  gamma_pid_.i_state = configuration.gamma_i_state_;
  gamma_pid_.i_min = configuration.gamma_i_min_;
  gamma_pid_.i_max = configuration.gamma_i_max_;
  gamma_pid_.i_gain = configuration.gamma_i_gain_;
  gamma_pid_.p_gain = configuration.gamma_p_gain_;
  gamma_pid_.d_gain = configuration.gamma_d_gain_;

  gain_pid_.d_state = configuration.gain_;
  gain_pid_.i_state = configuration.gain_i_state_;
  gain_pid_.i_min = configuration.gain_i_min_;
  gain_pid_.i_max = configuration.gain_i_max_;
  gain_pid_.i_gain = configuration.gain_i_gain_;
  gain_pid_.p_gain = configuration.gain_p_gain_;
  gain_pid_.d_gain = configuration.gain_d_gain_;

  exposure_pid_.d_state = configuration.exposure_;
  exposure_pid_.i_state = configuration.exposure_i_state_;
  exposure_pid_.i_min = configuration.exposure_i_min_;
  exposure_pid_.i_max = configuration.exposure_i_max_;
  exposure_pid_.i_gain = configuration.exposure_i_gain_;
  exposure_pid_.p_gain = configuration.exposure_p_gain_;
  exposure_pid_.d_gain = configuration.exposure_d_gain_;

  saturation_pid_.d_state = configuration.saturation_;
  saturation_pid_.i_state = configuration.saturation_i_state_;
  saturation_pid_.i_min = configuration.saturation_i_min_;
  saturation_pid_.i_max = configuration.saturation_i_max_;
  saturation_pid_.i_gain = configuration.saturation_i_gain_;
  saturation_pid_.p_gain = configuration.saturation_p_gain_;
  saturation_pid_.d_gain = configuration.saturation_d_gain_;

  gain_lim_ = configuration.gain_lim_;
  exposure_lim_ = configuration.exposure_lim_;
  msv_lum_ = 0;
  msv_sat_ = 0;

  nh_.advertise<sonia_msgs::CameraFeatures>("camera/" +
      CameraConfiguration::name_ +
                                                "_features",
      1000);
}

//------------------------------------------------------------------------------
//
BaseCamera::~BaseCamera() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void BaseCamera::PublishCameraFeatures() const {
}

//------------------------------------------------------------------------------
//
void BaseCamera::SetFeature(const Feature &feat, double value) {
  std::stringstream ss;
  ss << std::hex << guid_ << " " << GetName();
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
  PublishCameraFeatures();
}

//------------------------------------------------------------------------------
//
double BaseCamera::GetFeature(const Feature &feat) const {
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
  auto l_hist = CalculateLuminanceHistogram(img);

  msv_lum_= CalculateMSV(l_hist, 5);
  try {
    ROS_INFO_STREAM("MSV: " << msv_lum_);

    const float msvUniform = 2.5f;

    if (msv_lum_ != msvUniform) {
      if (GetExposureValue() > exposure_lim_ && GetGainValue() > gain_lim_) {
        double error = fabs(GetGammaValue() - current_features_.gamma);
        current_features_.gamma = UpdatePID(gamma_pid_, error, GetGammaValue());
        SetGammaValue(current_features_.gamma);
        ROS_INFO_STREAM("Gamma: " << current_features_.gamma);
      } else if (GetGainValue() > gain_lim_) {
        double error = fabs(GetExposureValue() - current_features_.exposure);
        current_features_.exposure =
            UpdatePID(exposure_pid_, error, GetExposureValue());
        SetExposureValue(current_features_.exposure);
        ROS_INFO_STREAM("Exposure: " << current_features_.exposure);
      } else {
        double error = fabs(GetGainValue() - current_features_.gain);
        current_features_.gain = UpdatePID(gain_pid_, error, GetGainValue());
        SetGainValue(current_features_.gain);
        ROS_INFO_STREAM("Gain: " << current_features_.gain);
      }
    }
    if (msv_lum_ > 2 && msv_lum_ < 3) {
      auto s_hist = CalculateSaturationHistogram(img);
      msv_sat_ = CalculateMSV(s_hist, 5);
      if (msv_sat_ != msvUniform) {
        double error =
            fabs(GetSaturationValue() - current_features_.saturation);
        current_features_.saturation =
            UpdatePID(saturation_pid_, error, GetSaturationValue());
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
cv::Mat BaseCamera::CalculateLuminanceHistogram(const cv::Mat &img) const {
  static const int hist_size{256};
  static const float range[] = {0, 255};
  static const float *histRange{range};

  // Get the LUV image from the RGB image in input parameter.
  cv::Mat luvImg;
  cvtColor(img, luvImg, CV_RGB2Luv);

  // Splitting the LUV Image into 3 channels in the luv_planes.
  std::vector<cv::Mat> luv_planes;
  cv::split(luvImg, luv_planes);

  // Calculate the histogram of luminance by sending the first element of
  // the plane (the L channel)
  cv::Mat l_hist;
  cv::calcHist(&luv_planes[0], 1, 0, cv::Mat(), l_hist, 1, &hist_size,
               &histRange, true, false);
  return l_hist;
}

//------------------------------------------------------------------------------
//
cv::Mat BaseCamera::CalculateSaturationHistogram(const cv::Mat &img) const {
  static const int hist_size{256};
  static const float range[] = {0, 255};
  static const float *histRange{range};

  // Get the LUV image from the RGB image in input parameter.
  cv::Mat hsv_img;
  cvtColor(img, hsv_img, CV_RGB2HSV_FULL);

  // Splitting the LUV Image into 3 channels in the luv_planes.
  std::vector<cv::Mat> hsv_planes;
  cv::split(hsv_img, hsv_planes);

  // Calculate the histogram of saturation by sending the first element of
  // the plane (the L channel)
  cv::Mat hist;
  cv::calcHist(&hsv_planes[1], 1, 0, cv::Mat(), hist, 1, &hist_size, &histRange,
               true, false);
  return hist;
}

//------------------------------------------------------------------------------
//
float BaseCamera::CalculateMSV(const cv::Mat &img, int nbrRegion) {
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
  double p_term = 0, d_term = 0, i_term = 0;
  p_term = pid.p_gain * error;
  // calculate the proportional term
  // calculate the integral state with appropriate limiting
  pid.i_state += error;
  if (pid.i_state > pid.i_max)
    pid.i_state = pid.i_max;
  else if (pid.i_state < pid.i_min)
    pid.i_state = pid.i_min;
  i_term = pid.i_gain * pid.i_state;  // calculate the integral term
  d_term = pid.d_gain * fabs(pid.d_state - position);
  pid.d_state = position;
  return p_term + d_term + i_term;
}

}  // namespace provider_vision
