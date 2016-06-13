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

#ifndef PROVIDER_VISION_MEDIA_CAMERA_CONFIGURATION_H_
#define PROVIDER_VISION_MEDIA_CAMERA_CONFIGURATION_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/ros/configuration_parser.h>
#include <ros/node_handle.h>
#include <stdlib.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace provider_vision {

class CameraConfiguration : public atlas::ConfigurationParser {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CameraConfiguration>;
  using ConstPtr = std::shared_ptr<const CameraConfiguration>;
  using PtrList = std::vector<CameraConfiguration::Ptr>;
  using ConstPtrList = std::vector<CameraConfiguration::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit CameraConfiguration(const ros::NodeHandle &nh,
                               const std::string &name) ATLAS_NOEXCEPT;

  virtual ~CameraConfiguration();

  //==========================================================================
  // P U B L I C   M E M B E R S

  ros::NodeHandle nh_;

  uint64_t guid_;
  std::string name_;
  double framerate_;
  bool gain_auto_;
  bool shutter_auto_;
  bool exposure_auto_;
  bool white_balance_auto_;
  double gain_;
  double shutter_;
  double gamma_;
  double exposure_;
  double white_balance_blue_;
  double white_balance_red_;
  double saturation_;
  int width_;
  int height_;
  int x_offset_;
  int y_offset_;
  int format_;
  bool auto_brightness_auto_;
  int auto_brightness_target_;
  int auto_brightness_target_variation_;

  std::string undistortion_matrice_path_;

  //==========================================================================
  // P U B L I C   M E T H O D S

  std::string GetUndistortionMatricePath() const;

  void SetUndistortionMatricePath(const std::string path);

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void DeserializeConfiguration(const std::string &name);
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline void CameraConfiguration::SetUndistortionMatricePath(
    const std::string path) {
  undistortion_matrice_path_ = path;
}

//-----------------------------------------------------------------------------
//
inline std::string CameraConfiguration::GetUndistortionMatricePath() const {
  return undistortion_matrice_path_;
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_CONFIGURATION_H_
