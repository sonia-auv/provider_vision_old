/**
 * \file	base_camera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	19/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <ros/ros.h>
#include "provider_vision/media/camera/base_camera.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
BaseCamera::BaseCamera(const CameraConfiguration &configuration)
    : BaseMedia(configuration) {
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
  ss << std::hex << config_.GetGUID() << " " << GetName();

  switch (feat) {
    case Feature::SHUTTER:
      SetShutter(value);
      break;
    case Feature::SHUTTER_AUTO:

      if (value > 0) {
        SetShutterAuto(value);
      } else {
        //          //ROS_INFO_NAMED(ss.str(), "Setting shutter to manual");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::GAIN_AUTO:

      if (value > 0) {
        //          //ROS_INFO_NAMED(ss.str(), "Setting gain to auto");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        //          //ROS_INFO_NAMED(ss.str(), "Setting gain to manual");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::GAIN:
      ////ROS_INFO_NAMED(ss.str(), "Setting gain to manual %f", value);
      error =
          dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAIN, value);
      break;
    case Feature::FRAMERATE:
      ////ROS_INFO_NAMED(ss.str(), "Setting framerate to %f", value);
      error =
          dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   ConvertFramerateToEnum(value));
      uint32_t test;
      error = dc1394_feature_get_value(dc1394_camera_,
                                       DC1394_FEATURE_FRAME_RATE, &test);
      std::cout << "settting to : " << value << " got "
          << ConvertFramerateToEnum(test) << std::endl;
      break;
    case Feature::WHITE_BALANCE_AUTO:
      if (value > 0) {
        ////ROS_INFO_NAMED(ss.str(), "Setting auto white balance to auto");
        error = dc1394_feature_set_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ////ROS_INFO_NAMED(ss.str(), "Setting auto white balance to manual");
        error = dc1394_feature_set_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::WHITE_BALANCE_BLUE:
      ////ROS_INFO_NAMED(ss.str(), "Setting blue value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      blue = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(dc1394_camera_, blue, red);
      break;
    case Feature::WHITE_BALANCE_RED:
      ////ROS_INFO_NAMED(ss.str(), "Setting red value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      red = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(dc1394_camera_, blue, red);
      break;
    case Feature::ERROR_FEATURE:
    default:
      break;
  }

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("The media is not accessible");
  }
}

//------------------------------------------------------------------------------
//
float BaseCamera::GetFeature(const Feature &feat) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  dc1394feature_mode_t mode;
  uint32_t value;
  float return_val;

  try {
    switch (feat) {
      case Feature::SHUTTER:
        error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                         &value);
        break;
      case Feature::SHUTTER_AUTO:
        error = dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                        &mode);
        if (mode == DC1394_FEATURE_MODE_MANUAL)
          value = 0;
        else if (mode == DC1394_FEATURE_MODE_AUTO)
          value = 1;
        break;
      case Feature::FRAMERATE:
        error = dc1394_feature_get_value(dc1394_camera_,
                                         DC1394_FEATURE_FRAME_RATE, &value);
        return_val = ConvertFramerateToFloat(static_cast<unsigned int>(value));
        value = static_cast<uint32_t>(return_val);
        break;
      case Feature::WHITE_BALANCE_AUTO:
        error = dc1394_feature_get_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE, &mode);
        if (mode == DC1394_FEATURE_MODE_MANUAL)
          value = 0;
        else if (mode == DC1394_FEATURE_MODE_AUTO)
          value = 1;
        break;
      case Feature::WHITE_BALANCE_BLUE:
        error =
            dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
        value = blue;
        break;
      case Feature::WHITE_BALANCE_RED:
        error =
            dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
        value = red;
        break;
      case Feature::ERROR_FEATURE:
        break;
      case Feature::GAIN:
        return GetGainValue();
      case Feature::GAMMA:
        return GetGammaValue();
      case Feature::EXPOSURE:
        return GetExposureValue();
      case Feature::SATURATION:
        return GetSaturationValue();
      default:
        break;
    }
  } catch (const std::runtime_error &e) {
    ROS_ERROR(e.what());
  }

  if (error != DC1394_SUCCESS) {
    return -1.0f;
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
double BaseCamera::UpdatePID(const std::shared_ptr<SPid> &pid, double error, double position) ATLAS_NOEXCEPT
{
  double pTerm, dTerm, iTerm;
  pTerm = pid->pGain * error;
  // calculate the proportional term
  // calculate the integral state with appropriate limiting
  pid->iState += error;
  if (pid->iState > pid->iMax) pid->iState = pid->iMax;
  else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
  iTerm = pid->iGain * pid->iState; // calculate the integral term
  dTerm = pid->dGain * (pid->dState - position);
  pid->dState = position;
  return pTerm + dTerm + iTerm;
}

}  // namespace provider_vision
