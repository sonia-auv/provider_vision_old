/**
 * \file	ROSManager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	11/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "ros/ros_manager.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ROSManager::ROSManager(int argc, char **argv, std::string date_and_time)
    : _handle() {}

//------------------------------------------------------------------------------
//
ROSManager::~ROSManager() {}

}  // namespace vision_server