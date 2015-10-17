/**
 * \file	CAMConfig.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_CONFIG_H_
#define PROVIDER_VISION_CAM_CONFIG_H_

#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include "lib_atlas/sys/fsinfo.h"
#include "provider_vision/utils/pugixml.h"
#include "provider_vision/media/camera_configuration.h"
#include "provider_vision/utils/config.h"

namespace vision_server {

/**
 * Class to read and save ALL the cameras parameter of the system. It will
 * read/save it into xml format.
 * Each modification should be registered here, so we can remember it
 */
class ConfigurationHandler {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ConfigurationHandler>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ConfigurationHandler(const std::string &file);

  virtual ~ConfigurationHandler();

  //==========================================================================
  // P U B L I C   M E T H O D S

  std::vector<CameraConfiguration> ParseConfiguration();
  void SaveConfiguration(
      const std::vector<CameraConfiguration> &system_config);

 private:
  std::string file_;
};

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_CONFIG_H_
