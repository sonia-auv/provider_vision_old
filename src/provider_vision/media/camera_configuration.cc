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
    : ConfigurationParser(nh, "/provider_vision"),
      guid_(0),
      name_(name),
      framerate_(0),
      gain_auto_(false),
      shutter_auto_(false),
      exposure_auto_(false),
      white_balance_auto_(false),
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
      auto_brightness_auto_(true),
      auto_brightness_target_(128),
      auto_brightness_target_variation_(16) {
  DeserializeConfiguration(name);
}

//------------------------------------------------------------------------------
//
CameraConfiguration::~CameraConfiguration() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CameraConfiguration::DeserializeConfiguration(const std::string &name) {
  std::string guid_str = "";
  FindParameter(name + "_guid", guid_str);

  if (guid_str != "") {
    guid_ = std::stoull(guid_str);
  }

  FindParameter(name + "_name", name_);
  FindParameter(name + "_framerate", framerate_);
  FindParameter(name + "_gain_auto", gain_auto_);
  FindParameter(name + "_shutter_auto", shutter_auto_);
  FindParameter(name + "_white_balance_auto", white_balance_auto_);
  FindParameter(name + "_gain", gain_);
  FindParameter(name + "_shutter", shutter_);
  FindParameter(name + "_gamma", gamma_);
  FindParameter(name + "_exposure", exposure_);
  FindParameter(name + "_white_balance_blue", white_balance_blue_);
  FindParameter(name + "_white_balance_red", white_balance_red_);
  FindParameter(name + "_saturation", saturation_);
  FindParameter(name + "_width", width_);
  FindParameter(name + "_height", height_);
  FindParameter(name + "_x_offset", x_offset_);
  FindParameter(name + "_y_offset", y_offset_);
  FindParameter(name + "_format", format_);
  FindParameter(name + "_auto_brightness_auto", auto_brightness_auto_);
  FindParameter(name + "_auto_brightness_target", auto_brightness_target_);
  FindParameter(name + "_auto_brightness_target_variation",
                auto_brightness_target_variation_);
  FindParameter(name + "_exposure_auto", exposure_auto_);
}

}  // namespace provider_vision
