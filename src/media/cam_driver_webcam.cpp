/**
 * \file	CAMDriverWebcam.cpp
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
#include "media/cam_driver_webcam.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMDriverWebcam::CAMDriverWebcam(const CAMConfig config)
    : CAMDriver(config), DRIVER_TAG("[DC1394 Driver]"), _webcam(nullptr) {}

//------------------------------------------------------------------------------
//
CAMDriverWebcam::~CAMDriverWebcam() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CAMDriverWebcam::InitDriver() {
  _webcam = std::make_shared<CAMWebcam>();
  _camera_list.push_back(CameraID("Webcam", 0000000000000000));
}

//------------------------------------------------------------------------------
//
void CAMDriverWebcam::CloseDriver() {
  // delete _webcam;
  _webcam = nullptr;
  _camera_list.clear();
}

//------------------------------------------------------------------------------
//
bool CAMDriverWebcam::StartCamera(CameraID id) {
  if (_webcam != nullptr) {
    return _webcam->Start();
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool CAMDriverWebcam::StopCamera(CameraID id) {
  if (_webcam != nullptr) {
    return _webcam->Close();
  }
  return false;
}

//------------------------------------------------------------------------------
//
std::vector<CameraID> CAMDriverWebcam::GetCameraList() { return _camera_list; }

//------------------------------------------------------------------------------
//
bool CAMDriverWebcam::IsMyCamera(const std::string &nameMedia) {
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
std::shared_ptr<Media> CAMDriverWebcam::GetActiveCamera(CameraID id) { return _webcam; }

//------------------------------------------------------------------------------
//
void CAMDriverWebcam::SetFeature(FEATURE feat, CameraID id, float val) {
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
void CAMDriverWebcam::GetFeature(FEATURE feat, CameraID id, float &val) {
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
void CAMDriverWebcam::run() {}

//------------------------------------------------------------------------------
//
bool CAMDriverWebcam::WatchDogFunc() { return true; }

//------------------------------------------------------------------------------
//
void CAMDriverWebcam::PopulateCameraList() {}

}  // namespace vision_server
