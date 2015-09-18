/**
 * \file	DC1394Context.h
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
#include <dc1394/dc1394.h>

#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/media/camera/dc1394_camera.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * DC1394Context is a driver to access all cameras that can
 * be used by the library DC1394. It does firewire and some
 * USB camera (if follow IIDC 1.31)
 * It manage the bus, the active camera.
 * It is the one creating and destroying cameras.
 */
class DC1394Context : public BaseContext {
 public:
  const std::string DRIVER_TAG;
  const double TIME_FOR_BUS_ERROR = 3;

  //==========================================================================
  // P U B L I C   C / D T O R S

  DC1394Context();

  virtual ~DC1394Context();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void InitContext(const std::vector<CameraConfiguration> &cam_configuration_lists) override;

  void CloseContext() override;

  bool StartCamera(const std::string &name) override;

  bool StopCamera(const std::string &name) override;

  void SetFeature(BaseCamera::Feature feat, const std::string &name,
                  float val) override;

  void GetFeature(BaseCamera::Feature feat, const std::string &name,
                  float &val) const override;

  bool IsMyCamera(const std::string &nameMedia) const override;

  void run() override;

  bool WatchDogFunc() override;

private:

  DC1394Camera &GetCameraFromMap(const std::string &name) const;
  DC1394Camera &GetCameraFromPair
    (const std::pair<std::string, std::shared_ptr<BaseMedia> > &pair) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  dc1394_t *_context;
};

//-----------------------------------------------------------------------------
//
DC1394Camera &DC1394Context::GetCameraFromMap(const std::string &name) const
{
  auto camera = camera_list_.find(name);
  if(camera != camera_list_.end())
  {
    throw std::invalid_argument("Camera does not exist");
  }
  return GetCameraFromPair(*camera);
}

//-----------------------------------------------------------------------------
//
DC1394Camera &DC1394Context::GetCameraFromPair
  (const std::pair<std::string, std::shared_ptr<BaseMedia> > &pair) const
{
  return dynamic_cast<DC1394Camera&>(*pair.second);
}

}  // namespace vision_server
#endif  // VISION_SERVER_CAM_DRIVER_DC1394_H_
