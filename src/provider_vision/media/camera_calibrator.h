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

#ifndef PROVIDER_VISION_MEDIA_CAMERA_CALIBRATOR_H_
#define PROVIDER_VISION_MEDIA_CAMERA_CALIBRATOR_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/ros/configuration_parser.h>
#include <ros/node_handle.h>
#include <stdlib.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace provider_vision {

class CameraCalibrator : public atlas::ConfigurationParser {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CameraCalibrator>;
  using ConstPtr = std::shared_ptr<const CameraCalibrator>;
  using PtrList = std::vector<CameraCalibrator::Ptr>;
  using ConstPtrList = std::vector<CameraCalibrator::ConstPtr>;

  struct PID {
    double i_state_;
    double i_min_;
    double i_max_;
    double i_gain_;
    double p_gain_;
    double d_gain_;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit CameraCalibrator(const ros::NodeHandle &nh,
                            const std::string &name);

  virtual ~CameraCalibrator();

  PID gamma_pid_;
  PID gain_pid_;
  PID exposure_pid_;
  PID saturation_pid_;

  double gain_lim_;
  double exposure_lim_;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void DeserializeConfiguration(const std::string &name) ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  std::string name_;
};

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_CALIBRATOR_H_
