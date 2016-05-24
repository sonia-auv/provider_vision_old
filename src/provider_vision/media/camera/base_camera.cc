/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include "provider_vision/media/camera/base_camera.h"
#include <ros/ros.h>
#include <sonia_msgs/CameraFeatures.h>
#include <type_traits>
#include <boost/any.hpp>

namespace provider_vision {

namespace {

inline bool CastToBool(const boost::any &op) {
  // Unfortunatly, cannot static assert the type of the value.
  // The type will be cast at runtime instead.
  try {
    return boost::any_cast<bool>(op);
  } catch (const boost::bad_any_cast &) {
    throw std::runtime_error("The value for this feature must be a boolean");
  }
}

inline double CastToDouble(const boost::any &op) {
  // Unfortunatly, cannot static assert the type of the value.
  // The type will be cast at runtime instead.
  try {
    return boost::any_cast<double>(op);
  } catch (const boost::bad_any_cast &) {
    throw std::runtime_error("The value for this feature must be a double");
  }
}
}

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
BaseCamera::BaseCamera(const CameraConfiguration &configuration)
    : BaseMedia(configuration.name_),
      CameraConfiguration(configuration),
      feature_pub_() {
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

  std::string base_node_name{kRosNodeName};
  feature_pub_ = nh_.advertise<sonia_msgs::CameraFeatures>(
      base_node_name + "/camera/" + CameraConfiguration::name_ + "_features",
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
  sonia_msgs::CameraFeatures msg;

  msg.shutter_mode = static_cast<uint8_t>(GetShutterMode());
  msg.white_balance_mode = static_cast<uint8_t>(GetShutterMode());
  msg.gain_mode = static_cast<uint8_t>(GetShutterMode());

  msg.shutter_value = GetShutterValue();
  msg.gamma_value = GetShutterValue();
  msg.exposure_value = GetShutterValue();
  msg.saturation_value = GetShutterValue();
  msg.luminance_msv = GetLimunanceMSV();
  msg.saturation_msv = GetSaturationMSV();

  msg.gamma_pid = std::vector<double>{gamma_pid_.p_gain, gamma_pid_.i_gain,
                                      gamma_pid_.d_gain};
  msg.gain_pid =
      std::vector<double>{gain_pid_.p_gain, gain_pid_.i_gain, gain_pid_.d_gain};
  msg.exposure_pid = std::vector<double>{
      exposure_pid_.p_gain, exposure_pid_.i_gain, exposure_pid_.d_gain};
  msg.saturation_pid = std::vector<double>{
      saturation_pid_.p_gain, saturation_pid_.i_gain, saturation_pid_.d_gain};

  feature_pub_.publish(msg);
}

//------------------------------------------------------------------------------
//
void BaseCamera::SetFeature(const Feature &feat, const boost::any &value) {
  try {
    switch (feat) {
      case Feature::SHUTTER_VALUE:
        SetShutterValue(CastToDouble(value));
        break;
      case Feature::SHUTTER_MODE:
        SetShutterMode(CastToBool(value));
        break;
      case Feature::GAIN_MODE:
        SetGainMode(CastToBool(value));
        break;
      case Feature::GAIN_VALUE:
        SetGainValue(CastToDouble(value));
        break;
      case Feature::FRAMERATE_VALUE:
        SetFrameRateValue(CastToDouble(value));
        break;
      case Feature::WHITE_BALANCE_MODE:
        SetWhiteBalanceMode(CastToBool(value));
        break;
      case Feature::WHITE_BALANCE_BLUE_VALUE:
        SetWhiteBalanceBlueValue(CastToDouble(value));
        break;
      case Feature::WHITE_BALANCE_RED_VALUE:
        SetWhiteBalanceRedValue(CastToDouble(value));
        break;
      case Feature::EXPOSURE_VALUE:
        SetExposureValue(CastToDouble(value));
        break;
      case Feature::GAMMA_VALUE:
        SetGammaValue(CastToDouble(value));
        break;
      case Feature::SATURATION_VALUE:
        SetSaturationValue(CastToDouble(value));
        break;
      case Feature::EXPOSURE_MODE:
        SetExposureMode(CastToBool(value));
        break;
    }
  } catch (const std::runtime_error &e) {
    ROS_ERROR("Could not set the feature for the camera: %s", e.what());
    throw;
  }
  PublishCameraFeatures();
}

//------------------------------------------------------------------------------
//
void BaseCamera::GetFeature(const Feature &feat, boost::any &value) const {
  try {
    switch (feat) {
      case Feature::SHUTTER_VALUE:
        value = GetShutterValue();
        break;
      case Feature::SHUTTER_MODE:
        value = GetShutterMode();
        break;
      case Feature::FRAMERATE_VALUE:
        value = GetFrameRateValue();
        break;
      case Feature::WHITE_BALANCE_MODE:
        value = GetWhiteBalanceMode();
        break;
      case Feature::WHITE_BALANCE_BLUE_VALUE:
        value = GetWhiteBalanceBlue();
        break;
      case Feature::WHITE_BALANCE_RED_VALUE:
        value = GetWhiteBalanceRed();
        break;
      case Feature::GAIN_VALUE:
        value = GetGainValue();
        break;
      case Feature::GAMMA_VALUE:
        value = GetGammaValue();
        break;
      case Feature::EXPOSURE_VALUE:
        value = GetExposureValue();
        break;
      case Feature::SATURATION_VALUE:
        value = GetSaturationValue();
        break;
      case Feature::GAIN_MODE:
        value = GetGainMode();
        break;
      case Feature::EXPOSURE_MODE:
        value = GetExposureMode();
        break;
    }
  } catch (const std::runtime_error &e) {
    ROS_ERROR("Could not get the feature for the camera: %s", e.what());
    throw;
  }
}

//------------------------------------------------------------------------------
//
void BaseCamera::Calibrate(cv::Mat const &img) {
  std::lock_guard<std::mutex> guard(features_mutex_);

  auto l_hist = CalculateLuminanceHistogram(img);

  msv_lum_ = CalculateMSV(l_hist, 5);
  try {
    ROS_INFO_STREAM("MSV: " << msv_lum_);

    const float msvUniform = 2.5f;

    if (msv_lum_ != msvUniform) {
      if (GetExposureValue() < exposure_lim_ && GetGainValue() < gain_lim_) {
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

//------------------------------------------------------------------------------
//
float BaseCamera::GetLimunanceMSV() const noexcept {
  std::lock_guard<std::mutex> guard(features_mutex_);
  return msv_lum_;
}

//------------------------------------------------------------------------------
//
float BaseCamera::GetSaturationMSV() const noexcept {
  std::lock_guard<std::mutex> guard(features_mutex_);
  return msv_sat_;
}

}  // namespace provider_vision
