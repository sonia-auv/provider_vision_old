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

#include <provider_vision/media/context/file_context.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "provider_vision/media/camera/image_file.h"

namespace vision_server {

  //==============================================================================
  // C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

  //------------------------------------------------------------------------------
  //
  VideoFileContext::VideoFileContext()
  : BaseContext(),
    DRIVER_TAG("[MEDIA Driver]")
  {}

  //------------------------------------------------------------------------------
  //
  VideoFileContext::~VideoFileContext() {}

  //==============================================================================
  // M E T H O D   S E C T I O N

  //------------------------------------------------------------------------------
  //
  void
  VideoFileContext::InitContext(const std::vector<CameraConfiguration> &cam_configuration_lists)
  {
  }

  //------------------------------------------------------------------------------
  //
  void VideoFileContext::CloseContext() { camera_list_.clear(); }

  //------------------------------------------------------------------------------
  //
  bool VideoFileContext::StartCamera(const std::string &name) {

    // in the videoFile context, camera in list are existing camera.
    if(camera_list_.find(name) != camera_list_.end() )
      //already existing
      return true;

    MediaType type = GetMediaType(name);
    if( type == MediaType::IMAGE )
    {
      std::shared_ptr<ImageFile> file (new ImageFile(name));
      file->Start();
      camera_list_.insert( std::make_pair( name , file ) );
    }else if (type == MediaType::VIDEO )
    {
      std::shared_ptr<VideoFile> file (new VideoFile(name));
      file->Start();
      camera_list_.insert(std::make_pair(name, file));
    }else
    {
      throw std::invalid_argument("Not my camera type");
    }

    return true;
  }

  //------------------------------------------------------------------------------
  //
  bool VideoFileContext::StopCamera(const std::string &name) {

    // in the videoFile context, camera in list are existing camera.
    auto file = camera_list_.find(name);
    if(file == camera_list_.end() )
        //does not exist
        return true;

    bool result = (*file).second->Stop();
    camera_list_.erase(file);

    return result;
  }

  //------------------------------------------------------------------------------
  //
  bool VideoFileContext::IsMyCamera(const std::string &nameMedia) const {

    bool result = false;
    // cherche si la camera existe
    if(camera_list_.find(nameMedia) != camera_list_.end() )
    {
      //already existing
      result = true;
    }// N'existe pas dans le systeme, est-ce qu'on peut la creer?
    else if( GetMediaType(nameMedia) != MediaType::NONE)
    {
      // C'est un video ou une image
      result = true;
    }

    // N'est pas un video ni une image ou un media existant pour ce driver.
    return result;
  }
  //------------------------------------------------------------------------------
  //
  void VideoFileContext::SetFeature(BaseCamera::Feature feat, const std::string &name,
                                    float val)
  {
  }

  //------------------------------------------------------------------------------
  //
  void VideoFileContext::GetFeature(BaseCamera::Feature feat, const std::string &name,
                                    float &val) const
  {
  }

  //------------------------------------------------------------------------------
  //
  void VideoFileContext::run() {}

  //------------------------------------------------------------------------------
  //
  bool VideoFileContext::WatchDogFunc() { return true; }

  //------------------------------------------------------------------------------
  //
  VideoFileContext::MediaType VideoFileContext::GetMediaType(
      const std::string &nameMedia) const {
    // on commence par rechercher une image
    if (nameMedia.find(".jpg") != std::string::npos ||
        nameMedia.find(".png") != std::string::npos ||
        nameMedia.find(".bmp") != std::string::npos) {
      return MediaType::IMAGE;
    }

    // On n'a pas trouver d'image, on recherche une vidéo
    if (nameMedia.find(".avi") != std::string::npos ||
        nameMedia.find(".mp4") != std::string::npos) {
      return MediaType::VIDEO;
    }

    return MediaType::NONE;
  }

}  // namespace vision_server
