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
#include "config.h"

namespace vision_server {

typedef struct { CameraID camID; } CameraConfig;

//==============================================================================
// C L A S S E S

/**
 * Class to read and save ALL the cameras parameter of the system. It will
 * read/save it into xml format.
 * Each modification should be registered here, so we can remember it
 */
class CAMConfig {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  CAMConfig(std::string file);

  virtual ~CAMConfig();

  //==========================================================================
  // P U B L I C   M E T H O D S

  const std::vector<CameraConfig> GetConfigList();

  /**
   * Return a pointer to an cameraConfig structure.
   * Return nullptr if no config was found.
   */
  CameraConfig *GetConfig(uint64_t guid);

  CameraConfig *GetConfig(std::string name);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::vector<CameraConfig> _list;
};

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_CONFIG_H_