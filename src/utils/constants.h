/**
 * \file	ConstantAndType.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CONSTANTS_H_
#define VISION_SERVER_CONSTANTS_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <vision_server/image_feed.h>

#define VISION_NODE_NAME "/vision_server/"

namespace vision_server {

/**
 * When calling with node, so string, please use
 * the enum name, ex. "FRAMERATE" for FEATURE::FRAMERATE
 */
enum FEATURE {
  ERROR_FEATURE,
  SHUTTER_AUTO,
  SHUTTER,
  GAIN_AUTO,
  GAIN,
  WHITE_BALANCE_AUTO,
  WHITE_BALANCE_RED,
  WHITE_BALANCE_BLUE,
  FRAMERATE
};

enum STATUS { OPEN, CLOSE, STREAMING, ERROR };

const auto kConfigPath = getenv("SONIA_WORKSPACE_ROOT") +
                         std::string{"/src/provider_vision/config/"};

const auto kLogPath = getenv("SONIA_WORKSPACE_ROOT") + std::string{"/ros/log/"};

const auto kFilterchainExt = ".fc";

};  // namespace vision_server

#endif  // VISION_SERVER_CONSTANTS_H_
