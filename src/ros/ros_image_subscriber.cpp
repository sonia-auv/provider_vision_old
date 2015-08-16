/**
 * \file	ROSImageSubscriber.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include <lib_atlas/typedef.h>
#include "ros/ros_image_subscriber.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ROSImageSubscriber::ROSImageSubscriber(atlas::NodeHandlePtr node_handle,
                                       std::string topic_name)
    : _img_transport(*node_handle), _topic_name(std::move(topic_name)) {
  assert(node_handle.get() != nullptr);
  _topic_access.Create();
  _subscriber = _img_transport.subscribe(
      _topic_name, 1, &ROSImageSubscriber::imageCallback, this);
}

//------------------------------------------------------------------------------
//
ROSImageSubscriber::~ROSImageSubscriber() { _topic_access.Destroy(); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ROSImageSubscriber::GetImage(cv::Mat &image) { image = _image; }

//------------------------------------------------------------------------------
//
void ROSImageSubscriber::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, "bgr8");
    _image = ptr->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
  }
}

}  // namespace vision_server