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
#include <boost/any.hpp>
#include <type_traits>

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
      calibrator_(nh_, CameraConfiguration::name_),
      feature_pub_() {
  undistord_matrix_.InitMatrices(undistortion_matrice_path_);

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
  msg.luminance_msv = calibrator_.GetLimunanceMSV();
  msg.saturation_msv = calibrator_.GetSaturationMSV();

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

}  // namespace provider_vision
