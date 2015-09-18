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
#include <provider_vision/media/configuration_handler.h>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/camera/base_camera.h"

namespace vision_server {

  //==============================================================================
  // C L A S S E S

  /**
   * Base class for any media driver. It also provide a Camera interface
   * which enhance Media class' basic method with camera handling method.
   */
  class BaseContext: public atlas::Runnable {
  public:

    typedef std::map<std::string, std::shared_ptr<BaseMedia> > MediaMap;

    //==========================================================================
    // C O N S T R U C T O R S   A N D   D E S T R U C T O R

    /**
     * CTR/DSTR
     */
    BaseContext() = default;
    virtual ~BaseContext(){};

    /**
     * Method to Init and close a driver contains step needed by driver to have
     * a functioning environnement.
     */
    virtual void InitContext(const std::vector<CameraConfiguration> &cam_configuration_lists) = 0;

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
                            float &val) const  = 0;

    /**
     * Method to get all listed (connected) camera of the system
     */
    virtual std::vector<std::string> GetCameraList() const;

    /**
     * \return The camera if it is open.
     *         WILL NOT OPEN IT IF NOT.
     */
    virtual std::shared_ptr<BaseMedia> GetMedia(const std::string &name) const;

    /**
     * Utility to know if this ID is associated to the media.
     */
    virtual bool IsMyCamera(const std::string &nameMedia) const;

    virtual bool WatchDogFunc() = 0;

  protected:

    /**
     * Safe multithreading access
     */
    mutable std::mutex _driver_access;

    /**
     * Keeping track of medias.
     * List of all camera AVAILABLE in the system
     */
    MediaMap camera_list_;
  };

  //==============================================================================
  // I N L I N E   F U N C T I O N S   D E F I N I T I O N S

  //-----------------------------------------------------------------------------
  //
  inline bool
  BaseContext::IsMyCamera(const std::string &nameMedia) const
  {
    return camera_list_.find(nameMedia) != camera_list_.end();
  }

  //-----------------------------------------------------------------------------
  //
  inline std::vector<std::string>
  BaseContext::GetCameraList() const
  {
    std::vector<std::string> list;
    for(auto const &cam: camera_list_)
    {
      list.push_back(cam.first);
    }
    return list;
  }

  //-----------------------------------------------------------------------------
  //
  inline std::shared_ptr<BaseMedia>
  BaseContext::GetMedia(const std::string &name) const
  {
    auto camera = camera_list_.find(name);
    if( camera == camera_list_.end())
    {
      throw std::invalid_argument("Camera is not from this context.");
    }

    return (*camera).second;
  }

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_DRIVER_H_
