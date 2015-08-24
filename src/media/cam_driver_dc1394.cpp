/**
 * \file	CamDriverDC1394.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <vector>
#include <ros/ros.h>
#include "media/cam_driver_dc1394.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMDriverDC1394::CAMDriverDC1394(const CAMConfig config)
    : CAMDriver(config), DRIVER_TAG("[DC1394 Driver]"), _context(nullptr) {}

//------------------------------------------------------------------------------
//
CAMDriverDC1394::~CAMDriverDC1394() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CAMDriverDC1394::InitDriver() {
  _context = dc1394_new();
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing DC1394 driver");
  PopulateCameraList();
  ROS_INFO_NAMED(DRIVER_TAG, "Reseting cameras");
}

//------------------------------------------------------------------------------
//
void CAMDriverDC1394::CloseDriver() {
  dc1394_free(_context);
  _camera_list.clear();
}

//------------------------------------------------------------------------------
//
bool CAMDriverDC1394::StartCamera(CameraID id) {
  dc1394camera_t *camera_dc1394 = dc1394_camera_new(_context, id.GetGUID());
  auto cam = std::make_shared<CAMCameraDC1394>(camera_dc1394, id);

  if (cam->Open()) {
    if (cam->Start()) {
      _live_camera_list.push_back(cam);
      ROS_INFO_NAMED(DRIVER_TAG, "Successfully started %s", id.GetFullName());
      return true;
    } else {
      ROS_ERROR_NAMED(DRIVER_TAG, "Camera opened but not started started %s",
                      id.GetFullName());
    }
  } else {
    ROS_ERROR_NAMED(DRIVER_TAG, "DID NOT START %s", id.GetFullName());
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool CAMDriverDC1394::StopCamera(CameraID id) {
  std::shared_ptr<CAMCameraDC1394> cam(dynamic_cast<CAMCameraDC1394 *>(GetActiveCamera(id).get()));

  if (cam.get() == nullptr) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Could not find %s as active camera.",
                    id.GetFullName());
    return false;
  }
  cam->Stop();
  if (!cam->Close()) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Error closing %s", id.GetFullName());
    return false;
  }
  // retrieve element to remove it.
  for (int i = 0; i < _live_camera_list.size(); i++) {
    if (_live_camera_list[i]->GetCameraID().GetName() ==
        cam->GetCameraID().GetName())
      _live_camera_list.erase(_live_camera_list.begin() + i);
  }

  ROS_INFO_NAMED(DRIVER_TAG, "Successfully stopped %s", id.GetFullName());
  return false;
}

//------------------------------------------------------------------------------
//
std::vector<CameraID> CAMDriverDC1394::GetCameraList() {
  // Repopulate list to make sure we have all the cameras.
  PopulateCameraList();
  return _camera_list;
}

//------------------------------------------------------------------------------
//
bool CAMDriverDC1394::IsMyCamera(const std::string &nameMedia) {
  bool is_mine = false;
  auto iter = _camera_list.begin();
  std::vector<CameraID>::const_iterator end_iter = _camera_list.end();

  for (; iter != end_iter; iter++) {
    if (nameMedia == (*iter).GetName()) {
      is_mine = true;
      break;
    }
  }
  return is_mine;
}

//------------------------------------------------------------------------------
//
void CAMDriverDC1394::run() {
  while (!must_stop()) {
    atlas::SecTimer ::sleep(5);
    if (!WatchDogFunc()) {
      ROS_FATAL_NAMED(DRIVER_TAG, "=====   Entering camera reset bus =====");
      std::vector<std::shared_ptr<CAMCameraDC1394>> cam_to_restart;
      CameraID tempID;

      for (auto &elem : _live_camera_list) {
        tempID = elem->GetCameraID();

        std::shared_ptr<CAMCameraDC1394> cam(dynamic_cast<CAMCameraDC1394 *>(elem.get()));
        cam_to_restart.push_back(cam);

        if (cam.get() != nullptr) {
          cam->Stop();
          ROS_FATAL_NAMED(DRIVER_TAG, "Reseting bus on %s",
                          tempID.GetFullName());
          dc1394_reset_bus(cam->GetCameraPtr());
          atlas::SecTimer ::sleep(1);
        } else {
          ROS_FATAL_NAMED(DRIVER_TAG, "Missed cast on %s",
                          tempID.GetFullName());
        }
      }
      atlas::SecTimer ::sleep(2);
      ROS_INFO_NAMED(DRIVER_TAG, "Restarting the camera.");
      // Restarting the camera is done after everything, since resetting
      // the bus
      // is
      // rude to other camera. We ensure a certain "safety" by doing so...
      for (auto &elem : cam_to_restart) {
        elem->Start();
      }
      atlas::SecTimer ::sleep(2);
    }
  }
}

//------------------------------------------------------------------------------
//
bool CAMDriverDC1394::WatchDogFunc() {
  bool fail = true;
  for (auto &elem : _live_camera_list) {
    CameraID tempID = elem->GetCameraID();
    std::shared_ptr<CAMCameraDC1394> cam(dynamic_cast<CAMCameraDC1394 *>(elem.get()));
    if (cam.get() != nullptr) {
      // If it has been 3 second no images have been produce
      if (cam->GetAcquistionTimerValue() > 3) {
        ROS_FATAL_NAMED(DRIVER_TAG, "Camera %s is not feeding!",
                        tempID.GetFullName());
        fail = false;
      }
    }
  }

  return fail;
}

//------------------------------------------------------------------------------
//
void CAMDriverDC1394::PopulateCameraList() {
  dc1394error_t error;
  dc1394camera_list_t *list;
  // Clearing the list will cause issue when calling camera with a name
  // randomly
  // created...
  _camera_list.clear();

  error = dc1394_camera_enumerate(_context, &list);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Could not enumerate camera.");
    return;
  }

  if (list->num == 0) {
    ROS_WARN_NAMED(DRIVER_TAG, "No camera found.");
    return;
  }

  dc1394camera_t *camera;
  for (uint i = 0; i < list->num; i++) {
    camera = dc1394_camera_new(_context, list->ids[i].guid);

    if (!camera) {
      ROS_ERROR_NAMED(DRIVER_TAG, "Could not create camera with GUID %ld",
                      list->ids[i].guid);
      continue;
    }

    CameraConfig *config = _config.GetConfig(list->ids[i].guid);
    // Case no name found, create one (can't wait to see what it gives...)
    if (config != nullptr) {
      _camera_list.push_back(config->camID);
      ROS_INFO_NAMED(DRIVER_TAG, "Found camera: %s",
                     config->camID.GetFullName());
    }

    dc1394_camera_free(camera);
  }

  dc1394_camera_free_list(list);
}

//------------------------------------------------------------------------------
//
std::string CAMDriverDC1394::GetNameFromGUID(uint64_t guid) {
  CameraConfig *cam_config = _config.GetConfig(guid);
  std::string name = "";
  if (cam_config) name = cam_config->camID.GetName();

  return name;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<Media> CAMDriverDC1394::GetActiveCamera(CameraID id) {
  auto iter = _live_camera_list.begin();
  auto end_iter = _live_camera_list.end();

  for (; iter != end_iter; iter++) {
    if ((*iter)->GetCameraID().GetGUID() == id.GetGUID()) {
      return (*iter);
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
void CAMDriverDC1394::SetFeature(FEATURE feat, CameraID id, float val) {
  std::shared_ptr<CAMCameraDC1394> cam(dynamic_cast<CAMCameraDC1394 *>(GetActiveCamera(id).get()));
  if (cam.get() != nullptr) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Camera is not active in the system.");
    return;
  }
  if (!cam->SetFeature(feat, val)) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Feature setting failed.");
  }
}

//------------------------------------------------------------------------------
//
void CAMDriverDC1394::GetFeature(FEATURE feat, CameraID id, float &val) {
  std::shared_ptr<CAMCameraDC1394> cam(dynamic_cast<CAMCameraDC1394 *>(GetActiveCamera(id).get()));
  if (cam.get() != nullptr) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Camera is not active in the system.");
    return;
  }
  val = cam->GetFeature(feat);
}

}  // namespace vision_server
