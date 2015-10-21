/**
 * \file	main.cpp
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <string>
#include "provider_vision/server/vision_server.h"

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(15);

  vision_server::VisionServer vision_server{};

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
