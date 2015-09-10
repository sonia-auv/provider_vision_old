/**
 * \file	VideoFileContext.cpp
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
#include <ros/ros.h>
#include "media/context/video_file_context.h"
#include "media/camera/image_file.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
VideoFileContext::VideoFileContext(const CAMConfig config)
    : BaseContext(config), DRIVER_TAG("[MEDIA Driver]") {}

//------------------------------------------------------------------------------
//
VideoFileContext::~VideoFileContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void VideoFileContext::InitDriver() {
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing MEDIA driver");
  PopulateCameraList();
  // note : les médias sont instancés en cours d'exécution, pas à la création
}

//------------------------------------------------------------------------------
//
void VideoFileContext::CloseDriver() { _live_camera_list.clear(); }

//------------------------------------------------------------------------------
//
bool VideoFileContext::StartCamera(CameraID id) {
  auto media = GetActiveCamera(id);
  if (media.get() == nullptr) {
    std::string nameMedia = id.GetName();
    // le media n'existe pas, donc on le créé
    if (GetMediaType(nameMedia) == IMAGE) {
      media = std::make_shared<ImageFile>(nameMedia);
    } else if (GetMediaType(nameMedia) == VIDEO) {
      media = std::make_shared<VideoFile>(nameMedia, true);
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
bool VideoFileContext::StopCamera(CameraID id) {
  auto media = GetActiveCamera(id);
  if (media.get() != nullptr) {
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
std::vector<CameraID> VideoFileContext::GetCameraList() {
  // feed la _camera_list (les données sont dans _live_camera_list)
  PopulateCameraList();
  return _camera_list;
}

//------------------------------------------------------------------------------
//
bool VideoFileContext::IsMyCamera(const std::string &nameMedia) {
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
std::shared_ptr<Media> VideoFileContext::GetActiveCamera(CameraID id) {
  for (auto &camera : _live_camera_list) {
    if (camera->GetCameraID().GetName() == id.GetName()) {
      return camera;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
void VideoFileContext::SetFeature(FEATURE feat, CameraID id, float val) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _live_camera_list) {
    // if(camera.GetGUID() == id.GetGUID() && _media != nullptr)
    //     _media->SetFeature(feat, val);
  }
}

//------------------------------------------------------------------------------
//
void VideoFileContext::GetFeature(FEATURE feat, CameraID id, float &val) {
  // Should not be necessary, but in case the driver has been close, the list
  // is empty so...
  for (const auto &camera : _live_camera_list) {
    // if (camera.GetGUID() == id.GetGUID() && _media != nullptr)
    //     val = _media->GetFeature(feat);
  }
}

//------------------------------------------------------------------------------
//
void VideoFileContext::run() {}

//------------------------------------------------------------------------------
//
bool VideoFileContext::WatchDogFunc() { return true; }

//------------------------------------------------------------------------------
//
void VideoFileContext::PopulateCameraList() {
  for (auto &camera : _live_camera_list) {
    _camera_list.push_back(camera->GetCameraID());
  }
}

//------------------------------------------------------------------------------
//
VideoFileContext::MEDIA_TYPE VideoFileContext::GetMediaType(
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
