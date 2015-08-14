/**
 * \file	ROSManager.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	11/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_ROS_MAMAGER_H_
#define VISION_SERVER_ROS_MAMAGER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <HTThread.h>
#include <ros/ros.h>
#include "ros/ros_image_topic.h"

namespace vision_server {

//==============================================================================
// G L O B A L   V A R I A B L E S   A N D   S T R U C T

static const std::string LOG_TOPIC_NAME = "vision_server_logger";

//==============================================================================
// C L A S S E S

/**
 * ROSManager handle general communication via ros.
 * It handles single node ros stuff (logger and command)
 * The image topic is created when needed, so it does not appear here.
 */
class ROSManager {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Need date_and_time for the log topic.
   */
  ROSManager(int argc, char **argv, std::string date_and_time);

  virtual ~ROSManager();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle _handle;
};

}  // namespace vision_server

#endif  // VISION_SERVER_ROS_MAMAGER_H_
