/**
 * \file	CamCameraDC1394.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_MEDIA_H_
#define VISION_SERVER_MEDIA_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <HTSmartPtr.h>
#include <opencv2/core/core.hpp>
#include "ros/ros_manager.h"
#include "utils/camera_id.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Base class for anything that can provide an image to the system
 * implement basic functionality that is called through the system.
 */
class Media : public HTSmartObj {
 public:
  std::vector<std::string> commands;

  CameraID _id;

  STATUS _status;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  typedef HTSmartPtr<Media> Ptr;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  Media(CameraID id) : _id(id), _status(CLOSE){};

  virtual ~Media(){};

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
  virtual bool NextImage(cv::Mat &image) = 0;

  /**
   * Return either if the camera is a real camera or not.
   * It will return false if, for exemple, the camera is a video or a Webcam.
   * Reimplement this method in your subclass and return the appropriate boolean
   *
   * \return True if the camera is a real camera.
   */
  virtual bool IsRealCamera() const = 0;

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Returns the current camera status
   */
  virtual STATUS getStatus() const;

  /**
   * Return the list of command available for this media
   */
  virtual std::vector<std::string> getCommands() const;

  /**
   * Makes return true if it does not have a proper framerate
   * i.e. Images and video;
   */
  virtual bool HasArtificialFramerate();

  /**
   * Return the CameraID, the general identifier for a media in the system.
   */
  virtual const CameraID GetCameraID();
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline STATUS Media::getStatus() const { return _status; };

//------------------------------------------------------------------------------
//
inline std::vector<std::string> Media::getCommands() const { return commands; }

//------------------------------------------------------------------------------
//
inline bool Media::HasArtificialFramerate() { return true; }

//------------------------------------------------------------------------------
//
inline const CameraID Media::GetCameraID() { return _id; }

}  // namespace vision_server

#endif  // VISION_SERVER_MEDIA_H_
