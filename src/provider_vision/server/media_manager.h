/**
 * \file	MediaManager.h
 * \author  Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAMERA_MANAGER_H_
#define PROVIDER_VISION_CAMERA_MANAGER_H_

#include <memory>
#include <vector>
#include <bits/shared_ptr.h>
#include "provider_vision/media/base_media.h"
#include "provider_vision/media/camera/base_camera.h"
#include "provider_vision/media/context/base_context.h"

namespace vision_server {

class MediaManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MediaManager>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  MediaManager() noexcept;

  ~MediaManager() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void StartMedia(const std::string &media_name) noexcept;

  void StopMedia(const std::string &media) noexcept;

  BaseMedia::Ptr GetMedia(const std::string &name) const noexcept;

  std::vector<BaseMedia::Ptr> GetAllMedias() const noexcept;

  /**
   * If the media is a camera, set the feature to a specific value.
   *
   * This will try to convert the media to a camera. If this work, calls the
   * method to set a feature on it. If it does not, this throws an exception.
   *
   * \param media_name The name of the media to set the feature to.
   * \param feature The feature to change the value of.
   * \param value The value to set on the given feature.
   */
  void SetCameraFeature(const std::string &media_name,
                        const std::string &feature, float value);

  /**
   * If the media is a camera, get the value of the given feature.
   *
   * This will try to convert the media to a camera. If this work, calls the
   * method to ge the value of a feature on it. If it does not, this throws
   * an exception.
   *
   * \param media_name The name of the media to set the feature to.
   * \param feature The feature to change the value of.
   */
  float GetCameraFeature(const std::string &media_name,
                         const std::string &feature);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void InitializeContext();

  void CloseContext();

  BaseContext::Ptr GetContextFromMedia(const std::string &name) const;

  BaseCamera::Feature GetFeatureFromName(const std::string &name) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::vector<BaseContext::Ptr> contexts_;
};

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAMERA_MANAGER_H_
