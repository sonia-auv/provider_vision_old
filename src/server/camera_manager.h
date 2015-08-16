/**
 * \file	CameraManager.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAMERA_MANAGER_H_
#define VISION_SERVER_CAMERA_MANAGER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_atlas/ros/service_server_manager.h>
#include <vision_server/vision_server_get_media_param.h>
#include <vision_server/vision_server_set_media_param.h>
#include "config.h"
#include "media/cam_driver.h"
#include "media/cam_driver_dc1394.h"
#include "media/cam_driver_webcam.h"
#include "media/cam_config.h"
#include "media/cam_driver_media.h"
#include "server/acquisition_loop.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * CameraManager is the main manager for EVERY media.
 * It is responsible to know wich camera is in the system
 * by asking its drivers. It is also responsible of calling
 * the good driver to start/stop/[set/get]features.
 * It is also the provider of acquisition loop ptr.
 * It has the responsibility of creating/destructing them.
 */
class CameraManager : public atlas::ServiceServerManager<CameraManager> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  enum COMMAND { START, STOP, SET_FEATURE, GET_FEATURE };

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit CameraManager(const ros::NodeHandle &node_handle);

  virtual ~CameraManager();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Accumulate all the drivers' camear CameraID in a list.
   */
  std::vector<CameraID> GetCameraList();

  /**
   * Streaming command is realated to start/stop a media, and returing active
   * acquisition loop or closing it, depending of the command.
   */
  void StreammingCmd(COMMAND cmd, const std::string &mediaName,
                     AcquisitionLoop::Ptr &ptr);

  /**
   * ParametersCmd send  command of type feature to the media (shutter, white
   * balance...).
   */
  void ParametersCmd(COMMAND cmd, const std::string &mediaName, FEATURE feat,
                     float &val);

  /**
   * Change a String representing a parameter of FEATURE to the appropriate
   * enum, so it can be use in the system.
   */
  FEATURE NameToEnum(const std::string &name);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Calls the Init and Close method of each drivers in the list.
   * creation/destruction.
   */
  bool Init();

  bool Close();

  /**
   * Simple for loop iteration which pokes each driver to know if they
   * possess the camera asked for.
   */
  CAMDriver *GetDriverForCamera(const std::string &name);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * Answer to the service get media params
   */
  bool CallbackGetCMD(vision_server_get_media_param::Request &rqst,
                      vision_server_get_media_param::Response &rep);

  /**
   * Answer to the service asking to set a parameter of a media.
   */
  bool CallbackSetCMD(vision_server_set_media_param::Request &rqst,
                      vision_server_set_media_param::Response &rep);

  /**
   * Config object. To register/read cameras config.
   */
  CAMConfig _config;

  /**
   * List of the driers in the system
   */
  std::vector<CAMDriver *> _drivers;
};

}  // namespace vision_server

#endif  // VISION_SERVER_CAMERA_MANAGER_H_