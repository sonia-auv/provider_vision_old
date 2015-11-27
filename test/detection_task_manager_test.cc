/**
 * \file  detection_task_manager.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  22/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <sstream>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <ros/this_node.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "provider_vision/utils/config.h"
#include "provider_vision/server/detection_task_manager.h"
#include "provider_vision/server/media_manager.h"
#include "provider_vision/server/filterchain_manager.h"

std::shared_ptr<ros::NodeHandle> nh;

/**
 * Test class that subscribe to a topic and replace its image member with the
 * newest image sent by the ROS publisher.
 */
class TopicListener {
 public:
  /**
   * Constructor of the listener that takes the current node handler as well as
   * the name of the topic to subscribe to.
   */
  explicit TopicListener(const std::string &topic_name)
      : image_transport(*nh) {
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    subscriber = image_transport.subscribe(
        topic_name, 1, &TopicListener::MessageCallBack, this, hints);
  }

  /**
   * Callback that is used by ros to give us the image message.
   */
  void MessageCallBack(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image = ptr->image;
    cv::imshow("test", image);
    cv::waitKey(0);
  }

  /**
   * Run while ROS is running
   */
  void Run() {
    while(ros::ok()) {
      ros::spinOnce();
    }
  }

  /**
   * Last image getter
   */
  const cv::Mat &GetImage() const {
    return image;
  }

 private:
  cv::Mat image;
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber subscriber;
};

TEST(DetectionTaskManager, start_detection) {
  provider_vision::FilterchainManager fmgr;
  auto fc = fmgr.InstanciateFilterchain("camera_feed");

  std::stringstream filepath;
  filepath << provider_vision::kProjectPath << "test/test_image.png";

  provider_vision::MediaManager mmgr;
  mmgr.OpenMedia(filepath.str());
  auto streamer = mmgr.StartStreamingMedia(filepath.str());

  provider_vision::DetectionTaskManager dmgr;
  dmgr.StartDetectionTask(streamer, fc, "test");

  // Checking that the detection has been created in the system.
  ASSERT_EQ(dmgr.GetAllDetectionTasksName().size(), 1);
  ASSERT_EQ(*(dmgr.GetAllDetectionTasksName().begin()), "test");

  std::vector<std::string> nodes;
  ros::this_node::getAdvertisedTopics(nodes);
  std::stringstream topic_name;
  topic_name << provider_vision::kRosNodeName << "test" << "_result";

  // Check that a publisher has been created for this detection task
  ASSERT_NE(std::find(nodes.begin(), nodes.end(), topic_name.str()), nodes.end());
  ASSERT_NE(std::find(nodes.begin(), nodes.end(), provider_vision::kRosNodeName + "test" + "_image"), nodes.end());

  TopicListener listener(provider_vision::kRosNodeName + "test" + "_image");
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  auto origin = cv::imread(filepath.str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

  // Check if the image send on ROS is the same that the original one.
  // Allow some difference due to the compression.
  EXPECT_NEAR(cv::mean(listener.GetImage())[0], cv::mean(origin)[0], 0.1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nh = std::make_shared<ros::NodeHandle>("~");
  return RUN_ALL_TESTS();
}
