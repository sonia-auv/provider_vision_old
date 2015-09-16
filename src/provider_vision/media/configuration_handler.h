/**
 * \file	CAMConfig.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_CONFIG_H_
#define VISION_SERVER_CAM_CONFIG_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <iostream>

#include "utils/camera_id.h"
#include "provider_vision/media/camera_configuration.h"
#include "provider_vision/config.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Class to read and save ALL the cameras parameter of the system. It will
 * read/save it into xml format.
 * Each modification should be registered here, so we can remember it
 */
class ConfigurationHandler {
 public:
  //==========================================================================
  // P U B L I C   C / D T O R S

  ConfigurationHandler(const std::string &file);

  virtual ~ConfigurationHandler();

  //==========================================================================
  // P U B L I C   M E T H O D S

  std::map<std::string, CameraConfiguration> ParseConfiguration();
  void SaveConfiguration(
      const std::map<std::string, CameraConfiguration> &system_config) const;

 private:
  std::string file_;
};

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_CONFIG_H_
