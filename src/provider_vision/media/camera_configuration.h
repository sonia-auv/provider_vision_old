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
#include <ros/node_handle.h>
#include <stdlib.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace provider_vision {

class CameraConfiguration {
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

  /**
   * We want to define copy constructor here for optimization purpose.
   * Do not copy the members if is a rvalue.
   */
  explicit CameraConfiguration(const CameraConfiguration &rhs) ATLAS_NOEXCEPT;
  explicit CameraConfiguration(CameraConfiguration &&rhs) ATLAS_NOEXCEPT;

  virtual ~CameraConfiguration() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E M B E R S

  uint64_t guid_;
  std::string name_;
  double framerate_;
  bool gain_manual_;
  bool shutter_manual_;
  bool exposure_manual_;
  bool white_balance_manual_;
  double gain_;
  double shutter_;
  double gamma_;
  double exposure_;
  double white_balance_blue_;
  double white_balance_red_;
  double saturation_;
  double gamma_i_state_;
  double gamma_i_min_;
  double gamma_i_max_;
  double gamma_i_gain_;
  double gamma_p_gain_;
  double gamma_d_gain_;
  double gain_i_state_;
  double gain_i_min_;
  double gain_i_max_;
  double gain_i_gain_;
  double gain_p_gain_;
  double gain_d_gain_;
  double exposure_i_state_;
  double exposure_i_min_;
  double exposure_i_max_;
  double exposure_i_gain_;
  double exposure_p_gain_;
  double exposure_d_gain_;
  double saturation_i_state_;
  double saturation_i_min_;
  double saturation_i_max_;
  double saturation_i_gain_;
  double saturation_p_gain_;
  double saturation_d_gain_;
  double gain_lim_;
  double exposure_lim_;
  int width_;
  int height_;
  int x_offset_;
  int y_offset_;
  int format_;
  bool auto_brightness_;
  int auto_brightness_target_;
  int auto_brightness_target_variation_;

  ros::NodeHandle nh_;

  std::string undistortion_matrice_path_;

  //==========================================================================
  // P U B L I C   M E T H O D S

  std::string GetUndistortionMatricePath() const ATLAS_NOEXCEPT;

  void SetUndistortionMatricePath(const std::string path) ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void DeserializeConfiguration(const std::string &name) ATLAS_NOEXCEPT;

  template <typename Tp_>
  void FindParameter(const std::string &str, Tp_ &p) ATLAS_NOEXCEPT;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline void CameraConfiguration::SetUndistortionMatricePath(
    const std::string path) ATLAS_NOEXCEPT {
  undistortion_matrice_path_ = path;
}

//-----------------------------------------------------------------------------
//
inline std::string CameraConfiguration::GetUndistortionMatricePath() const
    ATLAS_NOEXCEPT {
  return undistortion_matrice_path_;
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_CONFIGURATION_H_
