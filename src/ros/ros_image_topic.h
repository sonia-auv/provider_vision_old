/**
 * \file	ROSImageTopic.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	11/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_ROS_IMAGE_TOPIC_H_
#define VISION_SERVER_ROS_IMAGE_TOPIC_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <CLMutex.h>
#include <HTSmartPtr.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include "utils/constants.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Class to broadcast image on an image topic.
 * This image can be read with ROSImageSubscriber class
 */
class ROSImageTopic : public HTSmartObj {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  typedef HTSmartPtr<ROSImageTopic> Ptr;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Needs an handle to create a topic and a topic_name so we
   * can identify it.
   */
  ROSImageTopic(ros::NodeHandle hdl, std::string topic_name);

  virtual ~ROSImageTopic();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Function to call to publish an image on a topic.
   */
  void PublishImage(cv::Mat &image);

  /**
    * Function to handle the creation and destruction of the topic.
   */
  void OpenTopic();

  void CloseTopic();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  image_transport::ImageTransport _img_transport;

  image_transport::Publisher _publisher;

  std::string _topic_name;

  CLMutex _topic_access;

  // Seems that the function to check if the publisher is valid doesn't really
  // work so...
  bool _valid;
};

//==========================================================================
// INLINE METHOD
inline void vision_server::ROSImageTopic::OpenTopic() {
  _topic_access.Create();
  _publisher = _img_transport.advertise(_topic_name, 1);
  _valid = true;
}

inline void vision_server::ROSImageTopic::CloseTopic() {
  _publisher.shutdown();
  _topic_access.Destroy();
  _valid = false;
}

}  // namespace vision_server

#endif  // VISION_SERVER_ROS_IMAGE_TOPIC_H_
