/**
 * \file	ROSImageSubscriber.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_ROS_IMAGE_SUBSCRIBER_H_
#define VISION_SERVER_ROS_IMAGE_SUBSCRIBER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <CLMutex.h>
#include <HTSmartPtr.h>
#include <vision_server/image_feed.h>
#include "config.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Class to subscribe to an image topic created with ROSImageTopic class.
 * This is mostly for the client side, so when sending a command to start a
 * stream, you should receive a name of topic, wich is the one to be used here.
 * Since it work with call back system, no need for thread.
 */
class ROSImageSubscriber : public image_transport::Subscriber,
                           public HTSmartObj {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  typedef HTSmartPtr<ROSImageSubscriber> Ptr;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Need the handle to subscribe and the topic to wich subscribe.
   */
  ROSImageSubscriber(atlas::NodeHandlePtr node_handle, std::string topic_name);

  virtual ~ROSImageSubscriber();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Function to call to get an image
   * TODO Subscriber system to prevent need for loops.
   */
  void GetImage(cv::Mat &image);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Function called when receiving images.
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  image_transport::ImageTransport _img_transport;

  image_transport::Subscriber _subscriber;

  std::string _topic_name;

  /**
   * Contains the most recent image.
   */
  cv::Mat _image;

  CLMutex _topic_access;
};

}  // namespace vision_server

#endif  // VISION_SERVER_ROS_IMAGE_SUBSCRIBER_H_
