/**
 * \file	camera_configuration.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/09/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

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
  bool white_balance_manual_;
  double gain_;
  double shutter_;
  double gamma_;
  double exposure_;
  double white_balance_blue_;
  double white_balance_red_;
  double saturation_;

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

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;

  std::string undistortion_matrice_path_;
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
