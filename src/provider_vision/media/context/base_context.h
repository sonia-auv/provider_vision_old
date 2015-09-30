/**
 * \file	CAMDriver.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_DRIVER_H_
#define PROVIDER_VISION_CAM_DRIVER_H_

#include <mutex>
#include <lib_atlas/pattern/runnable.h>
#include <provider_vision/media/configuration_handler.h>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/camera/base_camera.h"

namespace vision_server {

/**
 * Base class for any media driver. It also provide a Camera interface
 * which enhance Media class' basic method with camera handling method.
 */
class BaseContext : public atlas::Runnable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseContext>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  BaseContext() = default;

  virtual ~BaseContext(){};

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Method to Init and close a driver contains step needed by driver to have
   * a functioning environnement.
   */
  virtual void InitContext(
      const std::vector<CameraConfiguration> &cam_configuration_lists) = 0;

  virtual void CloseContext() = 0;

  /**
   * Method to handle cameras
   */
  virtual bool StartCamera(const std::string &name) = 0;

  virtual bool StopCamera(const std::string &name) = 0;

  /**
   * Feature setting/getting handler
   */
  virtual void SetFeature(BaseCamera::Feature feat, const std::string &name,
                          float val) = 0;

  virtual void GetFeature(BaseCamera::Feature feat, const std::string &name,
                          float &val) const = 0;

  /**
   * Method to get all listed (connected) camera of the system
   */
  virtual std::vector<std::shared_ptr<BaseMedia>> GetMediaList() const;

  /**
   * \return The camera if it is open.
   *         WILL NOT OPEN IT IF NOT.
   */
  virtual std::shared_ptr<BaseMedia> GetMedia(const std::string &name) const;

  /**
   * Utility to know if this ID is associated to the media.
   */
  virtual bool ContainsMedia(const std::string &nameMedia) const;

  virtual bool WatchDogFunc() = 0;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::vector<std::shared_ptr<BaseMedia>> media_list_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline bool BaseContext::ContainsMedia(const std::string &nameMedia) const {
  return GetMedia(nameMedia) != media_list_.end();
}

//-----------------------------------------------------------------------------
//
inline std::vector<std::shared_ptr<BaseMedia>> BaseContext::GetMediaList()
    const {
  return media_list_;
}

//-----------------------------------------------------------------------------
//
inline std::shared_ptr<BaseMedia> BaseContext::GetMedia(
    const std::string &name) const {
  auto camera = GetMedia(name);
  if (camera == media_list_.end()) {
    throw std::invalid_argument("Camera is not from this context.");
  }

  return (*camera).second;
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_DRIVER_H_
