/**
 * \file	main.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_atlas/ros/service_server_manager.h>
#include <ros/ros.h>
#include "provider_vision/server/vision_server.h"

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "provider_vision");
  ros::NodeHandle nh("~");

  provider_vision::VisionServer pv(nh);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
