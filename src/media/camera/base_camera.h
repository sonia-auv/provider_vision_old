/**
 * \file	camera.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	19/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAMERA_H_
#define VISION_SERVER_CAMERA_H_

//==============================================================================
// I N C L U D E   F I L E S

#include "config.h"
#include "media/camera/base_media.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Every camera (i.e. not video or image) should inherit this instead of
 * Media class.
 * This enables feature and open/close stuff.
 */
class BaseCamera : public BaseMedia {
 public:

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
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R
  BaseCamera (const CameraConfiguration &configuration);

  virtual ~BaseCamera ();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * For a camera, you can open the camera (i.e create it in the system)
   * but not make it stream (i.e. the hardware do not capture light)
   * therefore open and close are there to "make the camera exist in the
   * system"
   */
  virtual bool Open() = 0;

  virtual bool Close() = 0;

  virtual bool SetFeature(const Feature &feat, float value) = 0;

  virtual float GetFeature(const Feature &feat) const = 0;

  uint64_t GetGUID() const;

  bool HasArtificialFramerate() const override;

private:

  CameraUndistordMatrices undistord_matrix_;

};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool BaseCamera::HasArtificialFramerate() const { return false; }

//------------------------------------------------------------------------------
//
uint64_t BaseCamera::GetGUID() const { return config_.GetGUID(); };
}  // namespace vision_server

#endif  // VISION_SERVER_CAMERA_H_
