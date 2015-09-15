/**
 * \file	camera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	19/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "media/camera/base_camera.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
BaseCamera::BaseCamera(const CameraConfiguration &configuration)
: BaseMedia(configuration)
{
  undistord_matrix_.InitMatrices(config_.GetUndistortionMatricePath());
}

//------------------------------------------------------------------------------
//
BaseCamera::~BaseCamera() {}

}  // namespace vision_server

