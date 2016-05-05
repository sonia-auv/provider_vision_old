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
#include "provider_vision/config.h"
#include "provider_vision/server/detection_task_manager.h"
#include "provider_vision/server/media_manager.h"
#include "provider_vision/server/filterchain_manager.h"

ros::NodeHandle *nhp;

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
      : image_transport(*nhp),
        continue_(true) {
    subscriber = image_transport.subscribe(
        topic_name, 1, &TopicListener::MessageCallBack, this);
  }

  /**
   * Callback that is used by ros to give us the image message.
   */
  void MessageCallBack(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = ptr->image;
  }

  /**
   * Run while ROS is running
   */
  void Run() {
    while(ros::ok() && continue_) {
      ros::spinOnce();
    }
  }

  /**
   * Last image getter
   */
  const cv::Mat &GetImage() const {
    return image;
  }

  /**
   * Stop the execution of the Run method.
   */
  void Stop() noexcept {
    continue_ = false;
  }

 private:
  cv::Mat image;
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber subscriber;
  std::atomic_bool continue_;
};

TEST(DetectionTaskManager, start_detection) {
  provider_vision::FilterchainManager fmgr;
  auto fc = fmgr.InstanciateFilterchain("image_feed");

  std::stringstream filepath;
  filepath << provider_vision::kProjectPath << "test/img/test_image.png";

  provider_vision::MediaManager mmgr(*nhp);
  mmgr.OpenMedia(filepath.str());
  auto streamer = mmgr.StartStreamingMedia(filepath.str());

  provider_vision::DetectionTaskManager dmgr;
  dmgr.StartDetectionTask(streamer, fc, "test");

  // Checking that the detection has been created in the system.
  ASSERT_EQ(dmgr.GetAllDetectionTasksName().size(), 1);
  ASSERT_EQ(*(dmgr.GetAllDetectionTasksName().begin()), "test");

  // Check that starting exiting detection task throws.
  ASSERT_THROW(dmgr.StartDetectionTask(nullptr, nullptr, "test"), std::logic_error);

  // Check that starting exiting detection task throws.
  ASSERT_THROW(dmgr.StartDetectionTask(streamer, fc, ""), std::invalid_argument);

  std::vector<std::string> nodes;
  ros::this_node::getAdvertisedTopics(nodes);
  std::stringstream topic_name;
  topic_name << provider_vision::kRosNodeName << "test" << "_result";

  // Check that a publisher has been created for this detection task
  ASSERT_NE(std::find(nodes.begin(), nodes.end(), topic_name.str()), nodes.end());
  ASSERT_NE(std::find(nodes.begin(), nodes.end(), provider_vision::kRosNodeName + "test" + "_image"), nodes.end());

  TopicListener listener(provider_vision::kRosNodeName + "test" + "_image");
  std::thread thread(&TopicListener::Run, &listener);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto origin = cv::imread(filepath.str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

  // Check if the image send on ROS is the same that the original one.
  // Allow some difference due to the compression.
  ASSERT_EQ(cv::mean(listener.GetImage())[0], cv::mean(origin)[0]);
  listener.Stop();
  thread.join();
}

TEST(DetectionTaskManager, stop_detection) {
  provider_vision::FilterchainManager fmgr;
  auto fc = fmgr.InstanciateFilterchain("image_feed");

  std::stringstream filepath;
  filepath << provider_vision::kProjectPath << "test/img/test_image.png";

  provider_vision::MediaManager mmgr(*nhp);
  mmgr.OpenMedia(filepath.str());
  auto streamer = mmgr.StartStreamingMedia(filepath.str());

  provider_vision::DetectionTaskManager dmgr;
  dmgr.StartDetectionTask(streamer, fc, "test");
  dmgr.StopDetectionTask("test");

  // Checking that the detection has been created in the system.
  ASSERT_EQ(dmgr.GetAllDetectionTasksName().size(), 0);

  // Check that detection task didn't change behavior of streamer.
  ASSERT_TRUE(streamer->IsStreaming());

  // Check that stoping non existing detection task throws.
  ASSERT_THROW(dmgr.StopDetectionTask("test"), std::invalid_argument);
}

TEST(DetectionTaskManager, change_observer) {
  provider_vision::FilterchainManager fmgr;
  auto fc = fmgr.InstanciateFilterchain("camera_feed");

  std::stringstream filepath;
  filepath << provider_vision::kProjectPath << "test/img/test_image.png";

  provider_vision::MediaManager mmgr(*nhp);
  mmgr.OpenMedia(filepath.str());
  auto streamer = mmgr.StartStreamingMedia(filepath.str());

  provider_vision::DetectionTaskManager dmgr;
  dmgr.StartDetectionTask(streamer, fc, "test");

  TopicListener listener(provider_vision::kRosNodeName + "test" + "_image");

  std::thread thread(&TopicListener::Run, &listener);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  cv::Mat first_image;
  listener.GetImage().copyTo(first_image);

  dmgr.ChangeReturnImageToOrigin("test");
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  cv::Mat second_image;
  listener.GetImage().copyTo(second_image);

  // Check that the observer has changed.
  ASSERT_FALSE(std::equal(first_image.begin<uchar>(), first_image.end<uchar>(), second_image.begin<uchar>()));

  dmgr.ChangeReturnImageToFilterchain("test");
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  cv::Mat third_image;
  listener.GetImage().copyTo(third_image);

  // Check that the observer has changed to the filterchain output
  ASSERT_TRUE(std::equal(first_image.begin<uchar>(), first_image.end<uchar>(), third_image.begin<uchar>()));

  listener.Stop();
  thread.join();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
