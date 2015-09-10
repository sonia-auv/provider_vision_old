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
#include "media/context/webcam_context.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
WebcamContext::WebcamContext(const CAMConfig config)
    : BaseContext(config), DRIVER_TAG("[DC1394 Driver]"), _webcam(nullptr) {}

//------------------------------------------------------------------------------
//
WebcamContext::~WebcamContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void WebcamContext::InitDriver() {
  _webcam = std::make_shared<CAMWebcam>();
  _camera_list.push_back(CameraID("Webcam", 0000000000000000));
}

//------------------------------------------------------------------------------
//
void WebcamContext::CloseDriver() {
  // delete _webcam;
  _webcam = nullptr;
  _camera_list.clear();
}

//------------------------------------------------------------------------------
//
bool WebcamContext::StartCamera(CameraID id) {
  if (_webcam != nullptr) {
    return _webcam->Start();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool WebcamContext::StopCamera(CameraID id) {
  if (_webcam != nullptr) {
    return _webcam->Close();
  }
  return false;
}

//------------------------------------------------------------------------------
//
std::vector<CameraID> WebcamContext::GetCameraList() { return _camera_list; }

//------------------------------------------------------------------------------
//
bool WebcamContext::IsMyCamera(const std::string &nameMedia) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _camera_list) {
    if (camera.GetName() == nameMedia) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<Media> WebcamContext::GetActiveCamera(CameraID id) { return _webcam; }

//------------------------------------------------------------------------------
//
void WebcamContext::SetFeature(FEATURE feat, CameraID id, float val) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _camera_list) {
    if (camera.GetGUID() == id.GetGUID() && _webcam != nullptr) {
      _webcam->SetFeature(feat, val);
      break;
    }
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::GetFeature(FEATURE feat, CameraID id, float &val) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _camera_list) {
    if (camera.GetGUID() == id.GetGUID() && _webcam != nullptr) {
      val = _webcam->GetFeature(feat);
      break;
    }
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::run() {}

//------------------------------------------------------------------------------
//
bool WebcamContext::WatchDogFunc() { return true; }

//------------------------------------------------------------------------------
//
void WebcamContext::PopulateCameraList() {}

}  // namespace vision_server
