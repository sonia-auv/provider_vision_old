/*
 * CameraConfiguration.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: jeremie
 */

#include <media/camera_configuration.h>

CameraConfiguration::CameraConfiguration(const std::string &name)
    : guid_(0), name_(name) {}

CameraConfiguration::~CameraConfiguration() {}
