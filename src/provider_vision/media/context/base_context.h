/**
 * \file	CAMDriver.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_CONTEXT_BASE_CONTEXT_H_
#define PROVIDER_VISION_MEDIA_CONTEXT_BASE_CONTEXT_H_

#include <mutex>
#include <memory>
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

  virtual ~BaseContext() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Method to Init and close a driver contains step needed by driver to have
   * a functioning environnement.
   */
  virtual void InitContext(
      const std::vector<CameraConfiguration> &cam_configuration_lists) = 0;

  virtual void CloseContext() = 0;

  virtual void OpenMedia(const std::string &name) = 0;

  virtual void CloseMedia(const std::string &name) = 0;

  virtual void StartStreamingMedia(const std::string &name) = 0;

  virtual void StopStreamingMedia(const std::string &name) = 0;

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
  virtual std::vector<BaseMedia::Ptr> GetMediaList() const;

  /**
   * \return The camera if it is open.
   *         WILL NOT OPEN IT IF NOT.
   */
  virtual BaseMedia::Ptr GetMedia(const std::string &name) const;

  /**
   * Utility to know if this ID is associated to the media.
   */
  virtual bool ContainsMedia(const std::string &nameMedia) const;

  virtual bool WatchDogFunc() = 0;

 protected:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  virtual void EraseMedia(const std::string &name_media);

  std::vector<BaseMedia::Ptr> media_list_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline bool BaseContext::ContainsMedia(const std::string &nameMedia) const {
  if (GetMedia(nameMedia)) {
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
//
inline std::vector<BaseMedia::Ptr> BaseContext::GetMediaList() const {
  return media_list_;
}

//-----------------------------------------------------------------------------
//
inline BaseMedia::Ptr BaseContext::GetMedia(const std::string &name) const {
  BaseMedia::Ptr media(nullptr);

  for (auto &elem : media_list_) {
    if (elem.get()->GetName().compare(name) == 0) {
      media = elem;
    }
  }
  return media;
}

//-----------------------------------------------------------------------------
//
inline void BaseContext::EraseMedia(const std::string &name_media) {
  for (auto iter = media_list_.begin(); iter != media_list_.end(); iter++) {
    if (iter->get()->GetName().compare(name_media) == 0) {
      media_list_.erase(iter);
      return;
    }
  }
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_DRIVER_H_
