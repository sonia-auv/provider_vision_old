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

inline double CastToInt(const boost::any &op) {
  // Unfortunatly, cannot static assert the type of the value.
  // The type will be cast at runtime instead.
  try {
    return boost::any_cast<int>(op);
  } catch (const boost::bad_any_cast &) {
    throw std::runtime_error("The value for this feature must be a int");
  }
}
}

const double BaseCamera::INVALID_DOUBLE = DBL_MIN;
const float BaseCamera::INVALID_FLOAT = FLT_MIN;

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
BaseCamera::BaseCamera(const CameraConfiguration &configuration)
    : BaseMedia(configuration.name_),
      CameraConfiguration(configuration),
      feature_pub_()
// calibrator_(nh_, CameraConfiguration::name_)
{
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

  bool tmp_bool;
  GetShutterMode(tmp_bool);
  msg.shutter_auto = static_cast<uint8_t>(tmp_bool);
  GetShutterMode(tmp_bool);
  msg.white_balance_auto = static_cast<uint8_t>(tmp_bool);
  GetShutterMode(tmp_bool);
  msg.gain_auto = static_cast<uint8_t>(tmp_bool);

  double tmp_val;
  GetGainValue(tmp_val);
  msg.gain = tmp_val;
  GetGammaValue(tmp_val);
  msg.gamma = tmp_val;
  GetExposureValue(tmp_val);
  msg.exposure = tmp_val;
  GetSaturationValue(tmp_val);
  msg.saturation = tmp_val;

  //  msg.luminance_msv = calibrator_.GetLimunanceMSV();
  //  msg.saturation_msv = calibrator_.GetSaturationMSV();

  feature_pub_.publish(msg);
}

//------------------------------------------------------------------------------
//
bool BaseCamera::SetFeature(const Feature &feat, const boost::any &value) {
  bool result = false;
  switch (feat) {
    case Feature::SHUTTER_VALUE:
      result = SetShutterValue(CastToDouble(value));
      break;
    case Feature::SHUTTER_AUTO:
      result = SetShutterMode(CastToBool(value));
      break;
    case Feature::GAIN_AUTO:
      result = SetGainMode(CastToBool(value));
      break;
    case Feature::GAIN_VALUE:
      result = SetGainValue(CastToDouble(value));
      break;
    case Feature::FRAMERATE_VALUE:
      result = SetFrameRateValue(CastToDouble(value));
      break;
    case Feature::WHITE_BALANCE_AUTO:
      result = SetWhiteBalanceMode(CastToBool(value));
      break;
    case Feature::WHITE_BALANCE_BLUE_VALUE:
      result = SetWhiteBalanceBlueValue(CastToDouble(value));
      break;
    case Feature::WHITE_BALANCE_RED_VALUE:
      result = SetWhiteBalanceRedValue(CastToDouble(value));
      break;
    case Feature::WHITE_BALANCE_GREEN_VALUE:
      result = SetWhiteBalanceGreenValue(CastToDouble(value));
      break;
    case Feature::EXPOSURE_VALUE:
      result = SetExposureValue(CastToDouble(value));
      break;
    case Feature::GAMMA_VALUE:
      result = SetGammaValue(CastToDouble(value));
      break;
    case Feature::SATURATION_VALUE:
      result = SetSaturationValue(CastToDouble(value));
      break;
    case Feature::EXPOSURE_AUTO:
      result = SetExposureMode(CastToBool(value));
      break;
    case Feature::AUTOBRIGHTNESS_AUTO:
      result = SetAutoBrightnessMode(CastToBool(value));
      break;
    case Feature::AUTOBRIGHTNESS_TARGET:
      result = SetAutoBrightnessTarget(CastToInt(value));
      break;
    case Feature::AUTOBRIGHTNESS_VARIATION:
      result = SetAutoBrightnessTargetVariation(CastToInt(value));
      break;
    case Feature::WHITE_BALANCE_EXECUTE:
      result = SetWhiteBalanceMode(true);
      break;
    default:
      ROS_ERROR("Invalid feature given.");
      result = false;
  }
  if (!result) {
    ROS_ERROR("Could not set the feature for the camera: %s",
              media_name_.c_str());
  }
  // PublishCameraFeatures();
  return result;
}

//------------------------------------------------------------------------------
//
bool BaseCamera::GetFeature(const Feature &feat, boost::any &value) const {
  bool result;
  bool bool_val;
  double dbl_val;
  int int_val;
  switch (feat) {
    case Feature::SHUTTER_VALUE:
      result = GetShutterValue(dbl_val);
      value = dbl_val;
      break;
    case Feature::SHUTTER_AUTO:
      result = GetShutterMode(bool_val);
      value = bool_val;
      break;
    case Feature::FRAMERATE_VALUE:
      result = GetFrameRateValue(dbl_val);
      value = dbl_val;
      break;
    case Feature::WHITE_BALANCE_AUTO:
      result = GetWhiteBalanceMode(bool_val);
      value = bool_val;
      break;
    case Feature::WHITE_BALANCE_BLUE_VALUE:
      result = GetWhiteBalanceBlue(dbl_val);
      value = dbl_val;
      break;
    case Feature::WHITE_BALANCE_RED_VALUE:
      result = GetWhiteBalanceRed(dbl_val);
      value = dbl_val;
      break;
    case Feature::WHITE_BALANCE_GREEN_VALUE:
      result = GetWhiteBalanceGreen(dbl_val);
      value = dbl_val;
      break;
    case Feature::GAIN_VALUE:
      result = GetGainValue(dbl_val);
      value = dbl_val;
      break;
    case Feature::GAMMA_VALUE:
      result = GetGammaValue(dbl_val);
      value = dbl_val;
      break;
    case Feature::EXPOSURE_VALUE:
      result = GetExposureValue(dbl_val);
      value = dbl_val;
      break;
    case Feature::SATURATION_VALUE:
      result = GetSaturationValue(dbl_val);
      value = dbl_val;
      break;
    case Feature::GAIN_AUTO:
      result = GetGainMode(bool_val);
      value = bool_val;
      break;
    case Feature::EXPOSURE_AUTO:
      result = GetExposureMode(bool_val);
      value = bool_val;
      break;
    case Feature::AUTOBRIGHTNESS_AUTO:
      result = GetAutoBrightnessMode(bool_val);
      value = bool_val;
      break;
    case Feature::AUTOBRIGHTNESS_TARGET:
      result = GetAutoBrightnessTarget(int_val);
      value = int_val;
      break;
    case Feature::AUTOBRIGHTNESS_VARIATION:
      result = GetAutoBrightnessTargetVariation(int_val);
      value = int_val;
      break;
    default:
      ROS_ERROR("Feature invalid");
      break;
  }
  // For now, since we do have a system to detect error on getter, will ignore.
  // TODO Implement the system.
  return result;
}

}  // namespace provider_vision
