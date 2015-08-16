/**
 * \file	ROSImageTopic.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	11/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros_image_topic.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ROSImageTopic::ROSImageTopic(atlas::NodeHandlePtr node_handle,
                             std::string topic_name)
    : _img_transport(*node_handle),
      _topic_name(std::move(topic_name)),
      _valid(false) {
  assert(node_handle.get() != nullptr);
}

//------------------------------------------------------------------------------
//
ROSImageTopic::~ROSImageTopic() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_server::ROSImageTopic::PublishImage(cv::Mat &image) {
  if (!_valid) return;
  // std::cout << "Publishing..." << std::endl;
  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  CLMutex::Guard guard(_topic_access);
  _publisher.publish(msg);
}
}  // namespace vision_server
