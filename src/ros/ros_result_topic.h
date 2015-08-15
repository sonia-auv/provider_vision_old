/**
 * \file	ROSResultTopic.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	11/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_ROS_RESULT_TOPIC_H_
#define VISION_SERVER_ROS_RESULT_TOPIC_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <ros/ros.h>
#include <CLMutex.h>
#include <CLLog.h>
#include <CLDate.h>
#include <HTSmartPtr.h>
#include <vision_server/filterchain_return_message.h>
#include "config.h"

namespace vision_server {

//==============================================================================
// G L O B A L   V A R I A B L E S   A N D   S T R U C T

static const int RESULT_TOPIC_BUFFER_SIZE = 200;

//==============================================================================
// C L A S S E S

class ROSResultTopic : public ros::Publisher {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  typedef vision_server::filterchain_return_message MessageType;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  ROSResultTopic(std::string topic_full_name, ros::NodeHandle hdl);

  virtual ~ROSResultTopic();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Method to publish the string result
   */
  void PublishResultString(const std::string result);
};

}  // namespace vision_server

#endif  // VISION_SERVER_ROS_RESULT_TOPIC_H_
