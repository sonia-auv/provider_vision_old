/**
 * \file	main.cpp
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <string>
#include "server/vision_server.h"

//------------------------------------------------------------------------------
//
void RunUnitTestIfNeeded(int argc, char **argv) {
  TCHolder::RegisterAllObjects(argv[0]);
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "RunTest") {
      TCTestManager *testManager = TCTestManager::GetInstance();
      testManager->PerformUnitTests(std::string(argv[i + 1]), false);
      return;
    }
  }
}

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
  // Runs unit test if needed.
  RunUnitTestIfNeeded(argc, argv);

  ros::init(argc, argv, "vision_server");
  ros::NodeHandle nhl;
  ros::Rate loop_rate(15);

  vision_server::VisionServer vision_server{nhl};

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
