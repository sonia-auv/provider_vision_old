/**
 * \file	CAMDriverWebcam.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_DRIVER_WEBCAM_H_
#define VISION_SERVER_CAM_DRIVER_WEBCAM_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <opencv2/opencv.hpp>
#include "media/media.h"
#include "media/cam_driver.h"
#include "media/cam_webcam.h"
#include "config.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * CamDriverDC1394 is a driver to access all cameras that can be used by the
 * library DC1394. It does firewire and some USB camera (if follow IIDC 1.31)
 * It manage the bus, the active camera.
 * It is the one creating and destroying cameras.
 */
class CAMDriverWebcam : public CAMDriver {
 public:
  const std::string DRIVER_TAG;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  CAMDriverWebcam(const CAMConfig config);

  virtual ~CAMDriverWebcam();

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

  //==========================================================================
  // P R I V A T E   M E M B E R S
  CAMWebcam *_webcam;
};

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_DRIVER_WEBCAM_H_
