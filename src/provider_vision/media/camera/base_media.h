/**
 * \file	CamCameraDC1394.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_BaseMedia_H_
#define VISION_SERVER_BaseMedia_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <opencv2/core/core.hpp>
#include <media/camera_configuration.h>
#include "provider_vision/config.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Base class for anything that can provide an image to the system
 * implement basic functionality that is called through the system.
 */
class BaseMedia {
 public:
  enum class Status { OPEN, CLOSE, STREAMING, ERROR };

  //==========================================================================
  // P U B L I C   C / D T O R S

  BaseMedia(const CameraConfiguration &config)
      : config_(config), status_(Status::CLOSE){};

  virtual ~BaseMedia(){};

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Starts to get images
   */
  virtual bool Start() = 0;

  /**
   * Stop getting images
   */
  virtual bool Stop() = 0;

  /**
   * Gives the most recent image
   */
  virtual bool NextImage(const cv::Mat &image) = 0;

  /**
   * Return either if the camera is a real camera or not.
   * It will return false if, for exemple, the camera is a video or a Webcam.
   * Reimplement this method in your subclass and return the appropriate boolean
   *
   *
  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Returns the current camera Status
   */
  virtual Status GetStatus() const;

  /**
   * Makes return true if it does not have a proper framerate
   * i.e. Images and video;
   */
  virtual bool HasArtificialFramerate() const;

  /**
   * Return the CameraID, the general identifier for a BaseMedia in the system.
   */
  virtual const CameraConfiguration &GetCameraConfiguration() const;

  std::string GetName() const;

 private:
  CameraConfiguration config_;

  Status status_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline BaseMedia::Status BaseMedia::GetStatus() const { return status_; };

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
inline std::string BaseMedia::GetName() const { return config_.GetName(); }

}  // namespace vision_server

#endif  // VISION_SERVER_BaseMedia_H_
