/**
 * \file	main.cpp
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <signal.h>
#include <string>
#include "ros/ros_manager.h"
#include "server/vision_server.h"

bool stopLoop = false;

//------------------------------------------------------------------------------
//
void interupSignal(int dummy = 0) { stopLoop = true; }

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

  CLString dateString;
  CLDate dateObj;
  dateObj.GetDateAndTimeNoSpace(dateString);
  vision_server::ROSManager ros_manager(argc, argv, dateString);

  vision_server::VisionServer vs(ros_manager);

  signal(SIGINT, interupSignal);

  while (!stopLoop) {
    ros::spinOnce();
    CLTimer::Delay(15);
  }
  cv::waitKey(-1);

  return 1;
}
