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

#include <lib_atlas/config.h>

#define VISION_NODE_NAME "/vision_server/"

namespace vision_server {


const auto kProjectPath =
    atlas::kWorkspaceRoot + std::string{"/src/provider_vision/"};

const auto kConfigPath = kProjectPath + std::string{"/config/"};

const auto kFilterchainExt = ".fc";

};  // namespace vision_server

#endif  // VISION_SERVER_CONSTANTS_H_
