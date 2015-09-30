/**
 * \file	config.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CONFIG_H_
#define PROVIDER_VISION_CONFIG_H_

#include <lib_atlas/config.h>

namespace vision_server {

const std::string kRosNodeName = "/vision_server/";

const std::string kProjectPath =
    atlas::kWorkspaceRoot + "/src/provider_vision/";

const std::string kConfigPath = kProjectPath + "/config/";

const std::string kFilterchainExt = ".fc";

};  // namespace vision_server

#endif  // PROVIDER_VISION_CONFIG_H_
