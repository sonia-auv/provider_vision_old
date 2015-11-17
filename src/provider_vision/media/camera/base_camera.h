/**
 * \file	base_camera.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	19/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
#define PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_

#include <memory>
#include "provider_vision/utils/config.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/cam_undistord_matrices.h"

namespace provider_vision {

/**
 * Every camera (i.e. not video or image) should inherit this instead of
 * Media class.
 * This enables feature and open/close stuff.
 */
class BaseCamera : public BaseMedia {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseCamera>;

  /**
   * When calling with node, so string, please use
   * the enum name, ex. "FRAMERATE" for FEATURE::FRAMERATE
   */
  enum class Feature {
    ERROR_FEATURE,
    SHUTTER_AUTO,
    SHUTTER,
    GAIN_AUTO,
    GAIN,
    WHITE_BALANCE_AUTO,
    WHITE_BALANCE_RED,
    WHITE_BALANCE_BLUE,
    FRAMERATE
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseCamera(const CameraConfiguration &configuration);

  virtual ~BaseCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void SetFeature(const Feature &feat, float value) = 0;

  virtual float GetFeature(const Feature &feat) const = 0;

  bool HasArtificialFramerate() const override;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  CameraUndistordMatrices undistord_matrix_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool BaseCamera::HasArtificialFramerate() const { return false; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
