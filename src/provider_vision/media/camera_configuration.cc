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

#include <provider_vision/media/camera_configuration.h>
#include <boost/lexical_cast.hpp>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CameraConfiguration::CameraConfiguration(const ros::NodeHandle &nh,
                                         const std::string &name) ATLAS_NOEXCEPT
    : ConfigurationParser(nh, "/provider_vision/camera_parameters/"),
      guid_(0),
      name_(name),
      framerate_(0),
      gain_manual_(true),
      shutter_manual_(true),
      exposure_manual_(true),
      white_balance_manual_(true),
      gain_(350.0),
      shutter_(500.0),
      gamma_(0),
      exposure_(0),
      white_balance_blue_(511.0),
      white_balance_red_(412.0),
      saturation_(0),
      width_(480),
      height_(640),
      x_offset_(0),
      y_offset_(0),
      format_(17301513),
      auto_brightness_(true),
      auto_brightness_target_(128),
      auto_brightness_target_variation_(16) {
  DeserializeConfiguration(name);
}

//------------------------------------------------------------------------------
//
CameraConfiguration::~CameraConfiguration() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CameraConfiguration::DeserializeConfiguration(const std::string &name)
    ATLAS_NOEXCEPT {
  std::string guid_str = "";
  FindParameter("/guid", guid_str);

  if (guid_str != "") {
    guid_ = std::stoull(guid_str);
  }

  FindParameter(name + "/name", name_);
  FindParameter(name + "/framerate", framerate_);
  FindParameter(name + "/gain_manual", gain_manual_);
  FindParameter(name + "/shutter_manual",
                shutter_manual_);
  FindParameter(name + "/white_balance_manual",
                white_balance_manual_);
  FindParameter(name + "/gain", gain_);
  FindParameter(name + "/shutter", shutter_);
  FindParameter(name + "/gamma", gamma_);
  FindParameter(name + "/exposure", exposure_);
  FindParameter(name + "/white_balance_blue",
                white_balance_blue_);
  FindParameter(name + "/white_balance_red",
                white_balance_red_);
  FindParameter(name + "/saturation", saturation_);
  FindParameter(name + "/width", width_);
  FindParameter(name + "/height", height_);
  FindParameter(name + "/x_offset", x_offset_);
  FindParameter(name + "/y_offset", y_offset_);
  FindParameter(name + "/format", format_);
  FindParameter(name + "/auto_brightness",
                auto_brightness_);
  FindParameter(name + "/auto_brightness_target",
                auto_brightness_target_);
  FindParameter(
      name + "/auto_brightness_target_variation",
      auto_brightness_target_variation_);
  FindParameter(name + "/exposure_manual",
                exposure_manual_);
}

}  // namespace provider_vision
