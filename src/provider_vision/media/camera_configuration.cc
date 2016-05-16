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
      gamma_i_state_(0),
      gamma_i_min_(0),
      gamma_i_max_(0),
      gamma_i_gain_(0),
      gamma_p_gain_(0),
      gamma_d_gain_(0),
      gain_i_state_(0),
      gain_i_min_(0),
      gain_i_max_(0),
      gain_i_gain_(0),
      gain_p_gain_(0),
      gain_d_gain_(0),
      exposure_i_state_(0),
      exposure_i_min_(0),
      exposure_i_max_(0),
      exposure_i_gain_(0),
      exposure_p_gain_(0),
      exposure_d_gain_(0),
      saturation_i_state_(0),
      saturation_i_min_(0),
      saturation_i_max_(0),
      saturation_i_gain_(0),
      saturation_p_gain_(0),
      saturation_d_gain_(0),
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
  gamma_i_state_ = rhs.gamma_i_state_;
  gamma_i_max_ = rhs.gamma_i_max_;
  gamma_i_min_ = rhs.gamma_i_min_;
  gamma_i_gain_ = rhs.gamma_i_gain_;
  gamma_d_gain_ = rhs.gamma_d_gain_;
  gamma_p_gain_ = rhs.gamma_p_gain_;
  gain_i_state_ = rhs.gain_i_state_;
  gain_i_max_ = rhs.gain_i_max_;
  gain_i_min_ = rhs.gain_i_min_;
  gain_i_gain_ = rhs.gain_i_gain_;
  gain_d_gain_ = rhs.gain_d_gain_;
  gain_p_gain_ = rhs.gain_p_gain_;
  exposure_i_state_ = rhs.exposure_i_state_;
  exposure_i_max_ = rhs.exposure_i_max_;
  exposure_i_min_ = rhs.exposure_i_min_;
  exposure_i_gain_ = rhs.exposure_i_gain_;
  exposure_d_gain_ = rhs.exposure_d_gain_;
  exposure_p_gain_ = rhs.exposure_p_gain_;
  saturation_i_state_ = rhs.saturation_i_state_;
  saturation_i_max_ = rhs.saturation_i_max_;
  saturation_i_min_ = rhs.saturation_i_min_;
  saturation_i_gain_ = rhs.saturation_i_gain_;
  saturation_d_gain_ = rhs.saturation_d_gain_;
  saturation_p_gain_ = rhs.saturation_p_gain_;
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
  gamma_i_state_ = rhs.gamma_i_state_;
  gamma_i_max_ = rhs.gamma_i_max_;
  gamma_i_min_ = rhs.gamma_i_min_;
  gamma_i_gain_ = rhs.gamma_i_gain_;
  gamma_d_gain_ = rhs.gamma_d_gain_;
  gamma_p_gain_ = rhs.gamma_p_gain_;
  gain_i_state_ = rhs.gain_i_state_;
  gain_i_max_ = rhs.gain_i_max_;
  gain_i_min_ = rhs.gain_i_min_;
  gain_i_gain_ = rhs.gain_i_gain_;
  gain_d_gain_ = rhs.gain_d_gain_;
  gain_p_gain_ = rhs.gain_p_gain_;
  exposure_i_state_ = rhs.exposure_i_state_;
  exposure_i_max_ = rhs.exposure_i_max_;
  exposure_i_min_ = rhs.exposure_i_min_;
  exposure_i_gain_ = rhs.exposure_i_gain_;
  exposure_d_gain_ = rhs.exposure_d_gain_;
  exposure_p_gain_ = rhs.exposure_p_gain_;
  saturation_i_state_ = rhs.saturation_i_state_;
  saturation_i_max_ = rhs.saturation_i_max_;
  saturation_i_min_ = rhs.saturation_i_min_;
  saturation_i_gain_ = rhs.saturation_i_gain_;
  saturation_d_gain_ = rhs.saturation_d_gain_;
  saturation_p_gain_ = rhs.saturation_p_gain_;
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
  FindParameter("/camera_parameters/" + name + "/gamma_i_state",
                gamma_i_state_);
  FindParameter("/camera_parameters/" + name + "/gamma_i_min", gamma_i_min_);
  FindParameter("/camera_parameters/" + name + "/gamma_i_max", gamma_i_max_);
  FindParameter("/camera_parameters/" + name + "/gamma_i_gain", gamma_i_gain_);
  FindParameter("/camera_parameters/" + name + "/gamma_p_gain", gamma_p_gain_);
  FindParameter("/camera_parameters/" + name + "/gamma_d_gain", gamma_d_gain_);
  FindParameter("/camera_parameters/" + name + "/gamma_i_state",
                gamma_i_state_);
  FindParameter("/camera_parameters/" + name + "/gain_i_min", gain_i_min_);
  FindParameter("/camera_parameters/" + name + "/gain_i_max", gain_i_max_);
  FindParameter("/camera_parameters/" + name + "/gain_i_gain", gain_i_gain_);
  FindParameter("/camera_parameters/" + name + "/gain_p_gain", gain_p_gain_);
  FindParameter("/camera_parameters/" + name + "/gain_d_gain", gain_d_gain_);
  FindParameter("/camera_parameters/" + name + "/exposure_i_state",
                exposure_i_state_);
  FindParameter("/camera_parameters/" + name + "/exposure_i_min",
                exposure_i_min_);
  FindParameter("/camera_parameters/" + name + "/exposure_i_max",
                exposure_i_max_);
  FindParameter("/camera_parameters/" + name + "/exposure_i_gain",
                exposure_i_gain_);
  FindParameter("/camera_parameters/" + name + "/exposure_p_gain",
                exposure_p_gain_);
  FindParameter("/camera_parameters/" + name + "/exposure_d_gain",
                exposure_d_gain_);
  FindParameter("/camera_parameters/" + name + "/saturation_i_state",
                saturation_i_state_);
  FindParameter("/camera_parameters/" + name + "/saturation_i_min",
                saturation_i_min_);
  FindParameter("/camera_parameters/" + name + "/saturation_i_max",
                saturation_i_max_);
  FindParameter("/camera_parameters/" + name + "/saturation_i_gain",
                saturation_i_gain_);
  FindParameter("/camera_parameters/" + name + "/saturation_p_gain",
                saturation_p_gain_);
  FindParameter("/camera_parameters/" + name + "/saturation_d_gain",
                saturation_d_gain_);
  FindParameter("/camera_parameters/" + name + "/gain_lim", gain_lim_);
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
