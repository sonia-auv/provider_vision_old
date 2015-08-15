/**
 * \file	CamDriverDC1394.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_DRIVER_DC1394_H_
#define VISION_SERVER_CAM_DRIVER_DC1394_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <inttypes.h>

#include "media/media.h"
#include "media/cam_driver.h"
#include "media/cam_camera_dc1394.h"
#include "config.h"
#include "dc1394/dc1394.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * CamDriverDC1394 is a driver to access all cameras that can
 * be used by the library DC1394. It does firewire and some
 * USB camera (if follow IIDC 1.31)
 * It manage the bus, the active camera.
 * It is the one creating and destroying cameras.
 */
class CAMDriverDC1394 : public CAMDriver {
 public:
  const std::string DRIVER_TAG;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  CAMDriverDC1394(const CAMConfig config);

  virtual ~CAMDriverDC1394();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void InitDriver() override;

  void CloseDriver() override;

  bool StartCamera(CameraID id) override;

  bool StopCamera(CameraID id) override;

  std::vector<CameraID> GetCameraList() override;

  bool IsMyCamera(const std::string &nameMedia) override;

  Media::Ptr GetActiveCamera(CameraID id) override;

  void SetFeature(FEATURE feat, CameraID id, float val) override;

  void GetFeature(FEATURE feat, CameraID id, float &val) override;

  /**
   * HTThread override
   * Is traditionally use to call the watchdog.
   */
  void ThreadFunc() override;

  bool WatchDogFunc() override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PopulateCameraList() override;

  std::string GetNameFromGUID(uint64_t guid);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  dc1394_t *_context;
};

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_DRIVER_DC1394_H_
