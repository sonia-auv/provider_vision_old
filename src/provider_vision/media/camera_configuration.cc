/**
 * \file	camera_configuration.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/09/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <provider_vision/media/camera_configuration.h>
#include <boost/lexical_cast.hpp>

namespace provider_vision {

CameraConfiguration::CameraConfiguration(const ros::NodeHandle &nh,
                                         const std::string &name) ATLAS_NOEXCEPT
    : guid_(0),
      name_(name),
      framerate_(0),
      gain_manual_(true),
      shutter_manual_(true),
      white_balance_manual_(true),
      gain_(350.0),
      shutter_(500.0),
      gamma_(0),
      exposure_(0),
      white_balance_blue_(511.0),
      white_balance_red_(412.0),
      saturation_(0),
      gamma_iState_(0),
      gamma_iMin_(0),
      gamma_iMax_(0),
      gamma_iGain_(0),
      gamma_pGain_(0),
      gamma_dGain_(0),
      gain_iState_(0),
      gain_iMin_(0),
      gain_iMax_(0),
      gain_iGain_(0),
      gain_pGain_(0),
      gain_dGain_(0),
      exposure_iState_(0),
      exposure_iMin_(0),
      exposure_iMax_(0),
      exposure_iGain_(0),
      exposure_pGain_(0),
      exposure_dGain_(0),
      saturation_iState_(0),
      saturation_iMin_(0),
      saturation_iMax_(0),
      saturation_iGain_(0),
      saturation_pGain_(0),
      saturation_dGain_(0),
      gain_lim_(0),
      exposure_lim_(0),
      nh_(nh) {
  DeserializeConfiguration(name);
}

//------------------------------------------------------------------------------
//
CameraConfiguration::CameraConfiguration(const CameraConfiguration &rhs)
    ATLAS_NOEXCEPT {
  guid_ = rhs.guid_;
  name_ = rhs.name_;
  framerate_ = rhs.framerate_;
  gain_manual_ = rhs.gain_manual_;
  shutter_manual_ = rhs.shutter_manual_;
  white_balance_manual_ = rhs.white_balance_manual_;
  gain_ = rhs.gain_;
  shutter_ = rhs.shutter_;
  gamma_ = rhs.gamma_;
  exposure_ = rhs.exposure_;
  white_balance_blue_ = rhs.white_balance_blue_;
  white_balance_red_ = rhs.white_balance_red_;
  saturation_ = rhs.saturation_;
  gamma_iState_ = rhs.gamma_iState_;
  gamma_iMax_ = rhs.gamma_iMax_;
  gamma_iMin_ = rhs.gamma_iMin_;
  gamma_iGain_ = rhs.gamma_iGain_;
  gamma_dGain_ = rhs.gamma_dGain_;
  gamma_pGain_ = rhs.gamma_pGain_;
  gain_iState_ = rhs.gain_iState_;
  gain_iMax_ = rhs.gain_iMax_;
  gain_iMin_ = rhs.gain_iMin_;
  gain_iGain_ = rhs.gain_iGain_;
  gain_dGain_ = rhs.gain_dGain_;
  gain_pGain_ = rhs.gain_pGain_;
  exposure_iState_ = rhs.exposure_iState_;
  exposure_iMax_ = rhs.exposure_iMax_;
  exposure_iMin_ = rhs.exposure_iMin_;
  exposure_iGain_ = rhs.exposure_iGain_;
  exposure_dGain_ = rhs.exposure_dGain_;
  exposure_pGain_ = rhs.exposure_pGain_;
  saturation_iState_ = rhs.saturation_iState_;
  saturation_iMax_ = rhs.saturation_iMax_;
  saturation_iMin_ = rhs.saturation_iMin_;
  saturation_iGain_ = rhs.saturation_iGain_;
  saturation_dGain_ = rhs.saturation_dGain_;
  saturation_pGain_ = rhs.saturation_pGain_;
  gain_lim_ = rhs.gain_lim_;
  exposure_lim_ = rhs.exposure_lim_;
  nh_ = rhs.nh_;
}

//------------------------------------------------------------------------------
//
CameraConfiguration::CameraConfiguration(CameraConfiguration &&rhs)
    ATLAS_NOEXCEPT {
  guid_ = rhs.guid_;
  name_ = rhs.name_;
  framerate_ = rhs.framerate_;
  gain_manual_ = rhs.gain_manual_;
  shutter_manual_ = rhs.shutter_manual_;
  white_balance_manual_ = rhs.white_balance_manual_;
  gain_ = rhs.gain_;
  shutter_ = rhs.shutter_;
  gamma_ = rhs.gamma_;
  exposure_ = rhs.exposure_;
  white_balance_blue_ = rhs.white_balance_blue_;
  white_balance_red_ = rhs.white_balance_red_;
  saturation_ = rhs.saturation_;
  gamma_iState_ = rhs.gamma_iState_;
  gamma_iMax_ = rhs.gamma_iMax_;
  gamma_iMin_ = rhs.gamma_iMin_;
  gamma_iGain_ = rhs.gamma_iGain_;
  gamma_dGain_ = rhs.gamma_dGain_;
  gamma_pGain_ = rhs.gamma_pGain_;
  gain_iState_ = rhs.gain_iState_;
  gain_iMax_ = rhs.gain_iMax_;
  gain_iMin_ = rhs.gain_iMin_;
  gain_iGain_ = rhs.gain_iGain_;
  gain_dGain_ = rhs.gain_dGain_;
  gain_pGain_ = rhs.gain_pGain_;
  exposure_iState_ = rhs.exposure_iState_;
  exposure_iMax_ = rhs.exposure_iMax_;
  exposure_iMin_ = rhs.exposure_iMin_;
  exposure_iGain_ = rhs.exposure_iGain_;
  exposure_dGain_ = rhs.exposure_dGain_;
  exposure_pGain_ = rhs.exposure_pGain_;
  saturation_iState_ = rhs.saturation_iState_;
  saturation_iMax_ = rhs.saturation_iMax_;
  saturation_iMin_ = rhs.saturation_iMin_;
  saturation_iGain_ = rhs.saturation_iGain_;
  saturation_dGain_ = rhs.saturation_dGain_;
  saturation_pGain_ = rhs.saturation_pGain_;
  gain_lim_ = rhs.exposure_lim_;
  exposure_lim_ = rhs.exposure_lim_;
  nh_ = rhs.nh_;
}

CameraConfiguration::~CameraConfiguration() ATLAS_NOEXCEPT {}

//------------------------------------------------------------------------------
//
void CameraConfiguration::DeserializeConfiguration(const std::string &name)
    ATLAS_NOEXCEPT {
  std::string guid_str = "";
  FindParameter("/camera_parameters/" + name + "/guid", guid_str);

  if (guid_str != "") {
    guid_ = std::stoull(guid_str);
  }

  FindParameter("/camera_parameters/" + name + "/name", name_);
  FindParameter("/camera_parameters/" + name + "/framerate", framerate_);
  FindParameter("/camera_parameters/" + name + "/gain_manual", gain_manual_);
  FindParameter("/camera_parameters/" + name + "/shutter_manual",
                shutter_manual_);
  FindParameter("/camera_parameters/" + name + "/white_balance_manual",
                white_balance_manual_);
  FindParameter("/camera_parameters/" + name + "/gain", gain_);
  FindParameter("/camera_parameters/" + name + "/shutter", shutter_);
  FindParameter("/camera_parameters/" + name + "/gamma", gamma_);
  FindParameter("/camera_parameters/" + name + "/exposure", shutter_);
  FindParameter("/camera_parameters/" + name + "/white_balance_blue",
                white_balance_blue_);
  FindParameter("/camera_parameters/" + name + "/white_balance_red",
                white_balance_red_);
  FindParameter("/camera_parameters/" + name + "/saturation", saturation_);
  FindParameter("/camera_parameters/" + name + "/gamma_iState", gamma_iState_);
  FindParameter("/camera_parameters/" + name + "/gamma_iMin", gamma_iMin_);
  FindParameter("/camera_parameters/" + name + "/gamma_iMax", gamma_iMax_);
  FindParameter("/camera_parameters/" + name + "/gamma_iGain", gamma_iGain_);
  FindParameter("/camera_parameters/" + name + "/gamma_pGain", gamma_pGain_);
  FindParameter("/camera_parameters/" + name + "/gamma_dGain", gamma_dGain_);
  FindParameter("/camera_parameters/" + name + "/gamma_iState", gamma_iState_);
  FindParameter("/camera_parameters/" + name + "/gain_iMin", gain_iMin_);
  FindParameter("/camera_parameters/" + name + "/gain_iMax", gain_iMax_);
  FindParameter("/camera_parameters/" + name + "/gain_iGain", gain_iGain_);
  FindParameter("/camera_parameters/" + name + "/gain_pGain", gain_pGain_);
  FindParameter("/camera_parameters/" + name + "/gain_dGain", gain_dGain_);
  FindParameter("/camera_parameters/" + name + "/exposure_iState",
                exposure_iState_);
  FindParameter("/camera_parameters/" + name + "/exposure_iMin",
                exposure_iMin_);
  FindParameter("/camera_parameters/" + name + "/exposure_iMax",
                exposure_iMax_);
  FindParameter("/camera_parameters/" + name + "/exposure_iGain",
                exposure_iGain_);
  FindParameter("/camera_parameters/" + name + "/exposure_pGain",
                exposure_pGain_);
  FindParameter("/camera_parameters/" + name + "/exposure_dGain",
                exposure_dGain_);
  FindParameter("/camera_parameters/" + name + "/saturation_iState",
                saturation_iState_);
  FindParameter("/camera_parameters/" + name + "/saturation_iMin",
                saturation_iMin_);
  FindParameter("/camera_parameters/" + name + "/saturation_iMax",
                saturation_iMax_);
  FindParameter("/camera_parameters/" + name + "/saturation_iGain",
                saturation_iGain_);
  FindParameter("/camera_parameters/" + name + "/saturation_pGain",
                saturation_pGain_);
  FindParameter("/camera_parameters/" + name + "/saturation_dGain",
                saturation_dGain_);
  FindParameter("/camera_parameters/" + name + "/gian_lim", gain_lim_);
  FindParameter("/camera_parameters/" + name + "/exposure_lim", exposure_lim_);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
void CameraConfiguration::FindParameter(const std::string &str,
                                        Tp_ &p) ATLAS_NOEXCEPT {
  if (nh_.hasParam("/provider_vision" + str)) {
    nh_.getParam("/provider_vision" + str, p);
  } else {
    ROS_WARN_STREAM("Did not find /provider_vision" + str
                    << ". Using default value instead.");
  }
}

}  // namespace provider_vision
