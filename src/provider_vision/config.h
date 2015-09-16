/**
 * \file	config.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CONFIG_H_
#define VISION_SERVER_CONFIG_H_

#include <lib_atlas/config.h>

namespace vision_server {

constexpr std::string kRosNodeName = "/vision_server/";

constexpr std::string kProjectPath =
    atlas::kWorkspaceRoot + "/src/provider_vision/";

constexpr std::string kConfigPath = kProjectPath + "/config/";

constexpr std::string kFilterchainExt = ".fc";

};  // namespace vision_server

#endif  // VISION_SERVER_CONFIG_H_
