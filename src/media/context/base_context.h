/**
 * \file	CAMDriver.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_DRIVER_H_
#define VISION_SERVER_CAM_DRIVER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <mutex>
#include <lib_atlas/pattern/runnable.h>
#include <media/configuration_handler.h>
#include "config.h"
#include "media/camera/base_media.h"
#include "media/camera/base_camera.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Base class for any media driver. It also provide a Camera interface
 * which enhance Media class' basic method with camera handling method.
 */
class BaseContext: public atlas::Runnable {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * CTR/DSTR
   */
  BaseContext();

  virtual ~BaseContext();

  /**
   * Method to Init and close a driver contains step needed by driver to have
   * a functioning environnement.
   */
  virtual void InitDriver() = 0;

  virtual void CloseDriver() = 0;

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
                          float &val) = 0;

  /**
   * Method to get all listed (connected) camera of the system
   */
  virtual std::vector<CameraID> GetCameraList() = 0;

  /**
   * \return The camera if it is open.
   *         WILL NOT OPEN IT IF NOT.
   */
  virtual std::shared_ptr<BaseMedia> GetActiveCamera(CameraID id) = 0;

  virtual CameraID GetIDFromName(const std::string &name);

  /**
   * Utility to know if this ID is associated to the media.
   */
  virtual bool IsMyCamera(const std::string &nameMedia) = 0;

  /**
   * Method to refresh the camera list.
   */
  virtual void PopulateCameraList() = 0;

  virtual bool WatchDogFunc() = 0;

  /**
   * Config to list the cameras' parameters
   */
  CAMConfig _config;

  /**
   * Safe multithreading access
   */
  mutable std::mutex _driver_access;

  /**
   * Keeping track of medias.
   * List of all camera AVAILABLE in the system
   */
  std::vector<CameraID> _camera_list;

  /**
   * List of all camera ACTIVE in the system.
   * For camera, it is camera that are acquiring images.
   */
  std::vector<std::shared_ptr<BaseMedia>> _active_camera_list;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline CameraID BaseContext::GetIDFromName(const std::string &name) {
  for (auto &elem : _camera_list) {
    if (elem.GetName() == name) {
      return elem;
    }
  }
  return CameraID();
}

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_DRIVER_H_
