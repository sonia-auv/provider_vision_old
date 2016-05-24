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

#include <provider_vision/media/camera_calibrator.h>
#include <boost/lexical_cast.hpp>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CameraCalibrator::CameraCalibrator(const ros::NodeHandle &nh,
                                         const std::string &name)
    : ConfigurationParser(nh, "/provider_vision/camera_pid/"),
      name_(name),
      gamma_pid_(),
      gain_pid_(),
      exposure_pid_(),
      saturation_pid_(),
      gain_lim_(),
      exposure_lim_() {
  DeserializeConfiguration(name_);
}

//------------------------------------------------------------------------------
//
CameraCalibrator::~CameraCalibrator() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CameraCalibrator::DeserializeConfiguration(const std::string &name)
ATLAS_NOEXCEPT {
  FindParameter(name_ + "/gamma_i_state",
                gamma_pid_.i_state_);
  FindParameter(name_ + "/gamma_i_min", gamma_pid_.i_min_);
  FindParameter(name_ + "/gamma_i_max", gamma_pid_.i_max_);
  FindParameter(name_ + "/gamma_i_gain", gamma_pid_.i_gain_);
  FindParameter(name_ + "/gamma_p_gain", gamma_pid_.p_gain_);
  FindParameter(name_ + "/gamma_d_gain", gamma_pid_.d_gain_);
  FindParameter(name_ + "/gamma_i_state",
                gamma_pid_.i_state_);
  FindParameter(name_ + "/gain_i_min", gain_pid_.i_min_);
  FindParameter(name_ + "/gain_i_max", gain_pid_.i_max_);
  FindParameter(name_ + "/gain_i_gain", gain_pid_.i_gain_);
  FindParameter(name_ + "/gain_p_gain", gain_pid_.p_gain_);
  FindParameter(name_ + "/gain_d_gain", gain_pid_.d_gain_);
  FindParameter(name_ + "/exposure_i_state",
                exposure_pid_.i_state_);
  FindParameter(name_ + "/exposure_i_min",
                exposure_pid_.i_min_);
  FindParameter(name_ + "/exposure_i_max",
                exposure_pid_.i_max_);
  FindParameter(name_ + "/exposure_i_gain",
                exposure_pid_.i_gain_);
  FindParameter(name_ + "/exposure_p_gain",
                exposure_pid_.p_gain_);
  FindParameter(name_ + "/exposure_d_gain",
                exposure_pid_.d_gain_);
  FindParameter(name_ + "/saturation_i_state",
                saturation_pid_.i_state_);
  FindParameter(name_ + "/saturation_i_min",
                saturation_pid_.i_min_);
  FindParameter(name_ + "/saturation_i_max",
                saturation_pid_.i_max_);
  FindParameter(name_ + "/saturation_i_gain",
                saturation_pid_.i_gain_);
  FindParameter(name_ + "/saturation_p_gain",
                saturation_pid_.p_gain_);
  FindParameter(name_ + "/saturation_d_gain",
                saturation_pid_.d_gain_);
  FindParameter(name_ + "/gain_lim", gain_lim_);
  FindParameter(name_ + "/exposure_lim", exposure_lim_);
}

}  // namespace provider_vision
