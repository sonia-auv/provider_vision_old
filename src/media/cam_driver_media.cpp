/**
 * \file	CamDriverMedia.cpp
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
#include "media/cam_driver_media.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMDriverMedia::CAMDriverMedia(const CAMConfig config)
    : CAMDriver(config), DRIVER_TAG("[MEDIA Driver]") {}

//------------------------------------------------------------------------------
//
CAMDriverMedia::~CAMDriverMedia() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CAMDriverMedia::InitDriver() {
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing MEDIA driver");
  PopulateCameraList();
  // note : les médias sont instancés en cours d'exécution, pas à la création
}

//------------------------------------------------------------------------------
//
void CAMDriverMedia::CloseDriver() { _live_camera_list.clear(); }

//------------------------------------------------------------------------------
//
bool CAMDriverMedia::StartCamera(CameraID id) {
  auto media = GetActiveCamera(id);
  if (media.IsNull()) {
    std::string nameMedia = id.GetName();
    // le media n'existe pas, donc on le créé
    if (GetMediaType(nameMedia) == IMAGE) {
      media = new MMImage(nameMedia);
      if (media != nullptr) {
        media->_id = CameraID(nameMedia, 0);
        _live_camera_list.push_back(media);
      }
    } else if (GetMediaType(nameMedia) == VIDEO) {
      media = new MMVideo(nameMedia, true);
      if (media != nullptr) {
        media->_id = CameraID(nameMedia, 0);
        _live_camera_list.push_back(media);
      }
    } else {
      ROS_ERROR_NAMED(DRIVER_TAG, "Media not instanciate and not started %s",
                      id.GetFullName());
    }
    // on le start
    ROS_INFO_NAMED(DRIVER_TAG, "Successfully started %s", id.GetFullName());
  }

  return media->Start();
}

//------------------------------------------------------------------------------
//
bool CAMDriverMedia::StopCamera(CameraID id) {
  auto media = GetActiveCamera(id);
  if (media.IsNotNull()) {
    if (!media->Stop()) {
      ROS_ERROR_NAMED(DRIVER_TAG, "Error closing %s", id.GetFullName());
      return false;
    }

    // Retreive the camera and delete it.
    auto camera = _live_camera_list.begin();
    const auto camera_last = _live_camera_list.end();
    for (; camera != camera_last; ++camera) {
      if ((*camera)->GetCameraID().GetName() ==
          media->GetCameraID().GetName()) {
        _live_camera_list.erase(camera);
        return true;
      }
    }
  }
  ROS_ERROR_NAMED(DRIVER_TAG, "Could not find %s as active camera.",
                  id.GetFullName());
  return false;
}

//------------------------------------------------------------------------------
//
std::vector<CameraID> CAMDriverMedia::GetCameraList() {
  // feed la _camera_list (les données sont dans _live_camera_list)
  PopulateCameraList();
  return _camera_list;
}

//------------------------------------------------------------------------------
//
bool CAMDriverMedia::IsMyCamera(const std::string &nameMedia) {
  // cherche si la camera existe
  for (auto &camera : _live_camera_list) {
    if (camera->GetCameraID().GetName() == nameMedia) {
      return true;
    }
  }

  // N'existe pas dans le systeme, est-ce qu'on peut la creer?
  if (GetMediaType(nameMedia) != NONE) {
    // C'est un video ou une image
    return true;
  }

  // N'est pas un video ni une image ou un media existant pour ce driver.
  return false;
}

//------------------------------------------------------------------------------
//
Media::Ptr CAMDriverMedia::GetActiveCamera(CameraID id) {
  for (auto &camera : _live_camera_list) {
    if (camera->GetCameraID().GetName() == id.GetName()) {
      return camera;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
void CAMDriverMedia::SetFeature(FEATURE feat, CameraID id, float val) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _live_camera_list) {
    // if(camera.GetGUID() == id.GetGUID() && _media != nullptr)
    //     _media->SetFeature(feat, val);
  }
}

//------------------------------------------------------------------------------
//
void CAMDriverMedia::GetFeature(FEATURE feat, CameraID id, float &val) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _live_camera_list) {
    // if (camera.GetGUID() == id.GetGUID() && _media != nullptr)
    //     val = _media->GetFeature(feat);
  }
}

//------------------------------------------------------------------------------
//
void CAMDriverMedia::ThreadFunc() {}

//------------------------------------------------------------------------------
//
bool CAMDriverMedia::WatchDogFunc() { return true; }

//------------------------------------------------------------------------------
//
void CAMDriverMedia::PopulateCameraList() {
  for (auto &camera : _live_camera_list) {
    _camera_list.push_back(camera->GetCameraID());
  }
}

//------------------------------------------------------------------------------
//
CAMDriverMedia::MEDIA_TYPE CAMDriverMedia::GetMediaType(
    const std::string &nameMedia) {
  // on commence par rechercher une image
  if (nameMedia.find(".jpg") != std::string::npos ||
      nameMedia.find(".png") != std::string::npos ||
      nameMedia.find(".bmp") != std::string::npos) {
    return IMAGE;
  }

  // On n'a pas trouver d'image, on recherche une vidéo
  if (nameMedia.find(".avi") != std::string::npos ||
      nameMedia.find(".mp4") != std::string::npos) {
    return VIDEO;
  }

  return NONE;
}

}  // namespace vision_server
