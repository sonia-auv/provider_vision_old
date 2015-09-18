/*
 * CameraConfiguration.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: jeremie
 */

#include <provider_vision/media/camera_configuration.h>
namespace vision_server
{
  CameraConfiguration::CameraConfiguration(const std::string &name)
      : guid_(0), name_(name) {}

  CameraConfiguration::~CameraConfiguration() {}

}
