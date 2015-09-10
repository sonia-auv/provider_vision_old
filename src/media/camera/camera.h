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
#include "media/cam_config.h"
#include "media/media.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Every camera (i.e. not video or image) should inherit this instead of
 * Media class.
 * This enables feature and open/close stuff.
 */
class Camera : public Media {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R
  Camera(CameraID id);

  virtual ~Camera();

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

  virtual bool SetFeature(FEATURE feat, float value) = 0;

  virtual float GetFeature(FEATURE feat) = 0;

  bool HasArtificialFramerate() override;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool Camera::HasArtificialFramerate() { return false; }

}  // namespace vision_server

#endif  // VISION_SERVER_CAMERA_H_
