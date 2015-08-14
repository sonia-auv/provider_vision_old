/**
 * \file	ROSResultTopic.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	11/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "ros/ros_result_topic.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ROSResultTopic::ROSResultTopic(std::string topic_full_name, ros::NodeHandle hdl)
    : ros::Publisher(hdl.advertise<ROSResultTopic::MessageType>(
          topic_full_name.c_str(), RESULT_TOPIC_BUFFER_SIZE)) {}

//------------------------------------------------------------------------------
//
ROSResultTopic::~ROSResultTopic() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ROSResultTopic::PublishResultString(std::string result) {
  MessageType msg;
  msg.execution_result = result.c_str();
  publish(msg);
}

}  // namespace vision_server