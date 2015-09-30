/**
 * \file	WebcamContext.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <vector>
#include "provider_vision/media/context/webcam_context.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
WebcamContext::WebcamContext()
    : BaseContext(),
      DRIVER_TAG("[Webcam Driver]"),
      WEBCAM_NAME("Webcam"),
      webcam_() {}

//------------------------------------------------------------------------------
//
WebcamContext::~WebcamContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void WebcamContext::InitContext(
    const std::vector<CameraConfiguration> &cam_configuration_lists) {}

//------------------------------------------------------------------------------
//
void WebcamContext::CloseContext() {}

//------------------------------------------------------------------------------
//
bool WebcamContext::StartCamera(const std::string &name) {
  if (WEBCAM_NAME.compare(name) == 0) {
    return webcam_.Start();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool WebcamContext::StopCamera(const std::string &name) {
  if (WEBCAM_NAME.compare(name) == 0) {
    return webcam_.Close();
  }
  return false;
}

//------------------------------------------------------------------------------
//
void WebcamContext::SetFeature(BaseCamera::Feature feat,
                               const std::string &name, float val) {
  if (WEBCAM_NAME.compare(name) == 0) {
    webcam_.SetFeature(feat, val);
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::GetFeature(BaseCamera::Feature feat,
                               const std::string &name, float &val) const {
  if (WEBCAM_NAME.compare(name) == 0) {
    val = webcam_.GetFeature(feat);
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::run() {}

//------------------------------------------------------------------------------
//
bool WebcamContext::WatchDogFunc() { return true; }
}  // namespace vision_server
