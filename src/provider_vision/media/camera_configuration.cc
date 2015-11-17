/**
 * \file	camera_configuration.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/09/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */


#include <provider_vision/media/camera_configuration.h>
#include "provider_vision/utils/pugixml.h"

namespace provider_vision {
CameraConfiguration::CameraConfiguration(const std::string &name)
    : guid_(0), name_(name) {}

CameraConfiguration::~CameraConfiguration() {}
}
