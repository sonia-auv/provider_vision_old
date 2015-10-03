/**
 * \file	WebcamContext.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_DRIVER_WEBCAM_H_
#define PROVIDER_VISION_CAM_DRIVER_WEBCAM_H_

#include <opencv2/opencv.hpp>
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/media/camera/webcam_camera.h"
#include "provider_vision/config.h"

namespace vision_server {

/**
 * CamDriverDC1394 is a driver to access all cameras that can be used by the
 * library DC1394. It does firewire and some USB camera (if follow IIDC 1.31)
 * It manage the bus, the active camera.
 * It is the one creating and destroying cameras.
 */
class WebcamContext : public BaseContext {
 public:
  const std::string DRIVER_TAG;

  const std::string WEBCAM_NAME;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<WebcamContext>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  WebcamContext() noexcept;

  virtual ~WebcamContext() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void InitContext(
      const std::vector<CameraConfiguration> &cam_configuration_lists) override;

  void CloseContext() override;

  bool StartCamera(const std::string &name) override;

  bool StopCamera(const std::string &name) override;

  void SetFeature(BaseCamera::Feature feat, const std::string &name,
                  float val) override;

  void GetFeature(BaseCamera::Feature feat, const std::string &name,
                  float &val) const override;

  bool IsMyCamera(std::string &nameMedia) const;

  void Run() override;

  bool WatchDogFunc() override;

 private:
  WebcamCamera webcam_;
};

inline bool WebcamContext::IsMyCamera(std::string &nameMedia) const {
  return WEBCAM_NAME.compare(nameMedia) == 0;
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_DRIVER_WEBCAM_H_
