/**
 * \file	CameraID.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	18/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "provider_vision/utils/camera_id.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CameraID::CameraID() : _name(), _guid(0), _full_id_string() {}

//------------------------------------------------------------------------------
//
CameraID::CameraID(std::string name, uint64_t guid)
    : _name(name), _guid(guid), _full_id_string(name) {
  _full_id_string += "," + std::to_string(_guid);
}

//------------------------------------------------------------------------------
//
CameraID::~CameraID() {}

}  // namespace vision_server
