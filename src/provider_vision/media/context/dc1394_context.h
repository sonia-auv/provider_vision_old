/**
 * \file	DC1394Context.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_DRIVER_DC1394_H_
#define PROVIDER_VISION_CAM_DRIVER_DC1394_H_

#include <inttypes.h>
#include <dc1394/dc1394.h>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/media/camera/dc1394_camera.h"

namespace vision_server {

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
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DC1394Context>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  DC1394Context() noexcept;

  virtual ~DC1394Context() noexcept;

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

  bool ContainsMedia(const std::string &nameMedia) const override;

  void Run() override;

  bool WatchDogFunc() override;

 private:
  DC1394Camera::Ptr GetDC1394Camera(const std::string &name) const;
  DC1394Camera::Ptr GetDC1394Camera(BaseMedia::Ptr media) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S
  dc1394_t *_context;
};

//-----------------------------------------------------------------------------
//
inline bool
DC1394Context::ContainsMedia(const std::string &nameMedia) const
{
  for(const auto &cam : this->media_list_)
  {
    if( nameMedia.compare(cam->GetName())==0)
    {
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
//
inline DC1394Camera::Ptr
DC1394Context::GetDC1394Camera(const std::string &name) const
{
  return GetDC1394Camera(GetMedia(name));
}

//-----------------------------------------------------------------------------
//
inline DC1394Camera::Ptr
DC1394Context::GetDC1394Camera(BaseMedia::Ptr media) const
{
  DC1394Camera::Ptr tmp = std::dynamic_pointer_cast<DC1394Camera>(media);

  // Should not happen since if we get here, we are probably in a for
  // loop that iters through media_list_
  // OR we received a name which returned true at ContainsMedia call
  // since it is the first step for calling camera function on a context
  if(!tmp)
  {
    throw std::invalid_argument("Media is not a DC1394 camera");
  }
  return tmp;
}


}  // namespace vision_server
#endif  // PROVIDER_VISION_CAM_DRIVER_DC1394_H_
