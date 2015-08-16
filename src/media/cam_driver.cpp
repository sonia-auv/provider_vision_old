/**
 * \file	CAMDriver.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "media/cam_driver.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMDriver::CAMDriver(const CAMConfig config) : _config(config) {
}

//------------------------------------------------------------------------------
//
CAMDriver::~CAMDriver() {}

}  // namespace vision_server
