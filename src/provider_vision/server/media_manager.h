/**
 * \file	CameraManager.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAMERA_MANAGER_H_
#define PROVIDER_VISION_CAMERA_MANAGER_H_

#include <lib_atlas/ros/service_server_manager.h>
#include <provider_vision/media/configuration_handler.h>
#include "provider_vision/media/context/file_context.h"
#include "provider_vision/config.h"
#include "provider_vision/media/configuration_handler.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/media/context/dc1394_context.h"
#include "provider_vision/media/context/webcam_context.h"
#include "provider_vision/media/context/file_context.h"
#include "provider_vision/media/media_streamer.h"

namespace vision_server {

/**
 * MediaManager is the main manager for EVERY media.
 * It is responsible to know wich camera is in the system
 * by asking its drivers. It is also responsible of calling
 * the good driver to start/stop/[set/get]features.
 * It is also the provider of acquisition loop ptr.
 * It has the responsibility of creating/destructing them.
 */
class MediaManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MediaManager>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  MediaManager() noexcept;

  virtual ~MediaManager() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  MediaStreamer::Ptr StartCamera(
      const std::string &media_name) noexcept;

  void StopCamera(const std::string &media) noexcept;

  void SetFeature(const std::string &media_name, BaseCamera::Feature feat,
                  float val) noexcept;

  float GetFeature(const std::string &media_name,
                   BaseCamera::Feature feat) noexcept;

  BaseMedia::Ptr GetMedia(const std::string &name) const noexcept;

  /**
   * Accumulate all the drivers' camear CameraID in a list.
   */
  std::vector<BaseMedia::Ptr> GetAllMedias() const noexcept;

  /**
   * Change a String representing a parameter of FEATURE to the appropriate
   * enum, so it can be use in the system.
   */
  BaseCamera::Feature GetFeatureFromName(const std::string &name) const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Calls the Init and Close method of each drivers in the list.
   * creation/destruction.
   */
  void InitializeContext();

  void CloseContext();

  /**
   * Simple for loop iteration which pokes each driver to know if they
   * possess the camera asked for.
   */
  BaseContext::Ptr GetContextFromMedia(const std::string &name);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * List of the driers in the system
   */
  std::vector<BaseContext::Ptr> contexts_;
};

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAMERA_MANAGER_H_
