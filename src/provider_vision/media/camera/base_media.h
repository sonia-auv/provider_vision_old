/**
 * \file	CamCameraDC1394.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_BaseMedia_H_
#define PROVIDER_VISION_BaseMedia_H_

#include <memory>
#include <opencv2/core/core.hpp>
#include <provider_vision/media/camera_configuration.h>
#include "provider_vision/utils/config.h"

namespace vision_server {

/**
 * Base class for anything that can provide an image to the system
 * implement basic functionality that is called through the system.
 */
class BaseMedia {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseMedia>;

  enum class Status { OPEN, STREAMING, CLOSE, ERROR };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseMedia(const CameraConfiguration &config)
      : config_(config), status_(Status::CLOSE) {}

  virtual ~BaseMedia() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * For a camera, you can open the camera (i.e create it in the system)
   * but not make it stream (i.e. the hardware do not capture light)
   * therefore open and close are there to "make the camera exist in the
   * system"
   */
  virtual void Open() = 0;

  virtual void Close() = 0;

  /**
   * Starts to get images
   */
  void StartStreaming();

  /**
   * Stop getting images
   */
  void StopStreaming();

  /**
   * Gives the most recent image
   */
  virtual void NextImage(cv::Mat &image) = 0;

  /**
   * Call the next image and make a deep copy of it.
   * Warning, this method is very costly, use it only if you want the media to
   * keep the original image.
   */
  virtual void NextImageCopy(cv::Mat &image) noexcept;

  /**
   * Returns the current camera Status
   */
  virtual const Status &GetStatus() const;

  /**
   * Makes return true if it does not have a proper framerate
   * i.e. Images and video;
   */
  virtual bool HasArtificialFramerate() const;

  /**
   * Return the CameraID, the general identifier for a BaseMedia in the system.
   */
  virtual const CameraConfiguration &GetCameraConfiguration() const;

  const std::string &GetName() const;

  bool IsOpened() const;

  bool IsClosed() const;

  bool IsStreaming() const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Starts to get images
   */
  virtual void SetStreamingModeOn() = 0;

  /**
   * Stop getting images
   */
  virtual void SetStreamingModeOff() = 0;

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  CameraConfiguration config_;

  Status status_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void BaseMedia::NextImageCopy(cv::Mat &image) noexcept {
  cv::Mat tmp_image;
  NextImage(tmp_image);
  image = tmp_image.clone();
}

//------------------------------------------------------------------------------
//
inline const BaseMedia::Status &BaseMedia::GetStatus() const {
  return status_;
};

//------------------------------------------------------------------------------
//
inline bool BaseMedia::HasArtificialFramerate() const { return true; }

//------------------------------------------------------------------------------
//
inline const CameraConfiguration &BaseMedia::GetCameraConfiguration() const {
  return config_;
}

//------------------------------------------------------------------------------
//
inline const std::string &BaseMedia::GetName() const {
  return config_.GetName();
}

//------------------------------------------------------------------------------
//
inline bool BaseMedia::IsOpened() const { return Status::OPEN == status_; }

//------------------------------------------------------------------------------
//
inline bool BaseMedia::IsClosed() const { return Status::CLOSE == status_; }

//------------------------------------------------------------------------------
//
inline bool BaseMedia::IsStreaming() const {
  return Status::STREAMING == status_;
}

//------------------------------------------------------------------------------
//
inline void BaseMedia::StartStreaming() {
  if (GetStatus() == Status::OPEN) {
    SetStreamingModeOn();
  } else if (GetStatus() == Status::CLOSE) {
    Open();
    SetStreamingModeOn();
  } else if (GetStatus() == Status::STREAMING) {
    throw std::logic_error("The media is already streaming");
  } else {
    throw std::runtime_error(
        "The media is on an unstable status. Cannot stream.");
  }
}

//------------------------------------------------------------------------------
//
inline void BaseMedia::StopStreaming() {
  if (GetStatus() == Status::STREAMING) {
    SetStreamingModeOff();
  } else if (GetStatus() == Status::CLOSE) {
    throw std::logic_error("The media is not opened, cannot stop stream.");
  } else if (GetStatus() == Status::OPEN) {
    throw std::logic_error("The media is not streaming.");
  } else {
    throw std::runtime_error(
        "The media is on an unstable status. Cannot stream.");
  }
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_BaseMedia_H_
