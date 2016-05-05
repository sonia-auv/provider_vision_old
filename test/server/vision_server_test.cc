/**
 * \file  filterchain_manager_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  22/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */
#include <thread>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "provider_vision/config.h"
#include "provider_vision/server/media_manager.h"
#include <sonia_msgs/execute_cmd.h>
#include <provider_vision/server/vision_server.h>

ros::NodeHandle *nhp;

static const std::string node_prefix("/provider_vision/");
static const std::string test_dir(std::string(getenv("ROS_SONIA_WS"))
                                      + std::string("/src/provider_vision/test/"));

static bool stop_thread = false;

void RunVisionServer() {
  ros::Rate loop_rate(15);
  while (!stop_thread) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


/**
 * Test class that subscribe to a service and
 * send command to the vision server.
 */
class ServiceSubscriber {
 public:
  /**
   * Constructor of the listener that takes the current node handler as well as
   * the name of the topic to subscribe to.
   */
  explicit ServiceSubscriber(const std::string &topic_name) {
    ros::NodeHandle n;
    client_ = n.serviceClient<sonia_msgs::execute_cmd>(topic_name);
  }

  /**
   * Callback that is used by ros to give us the image message.
   */
  bool CallServer(const sonia_msgs::execute_cmdRequest &request,
                  sonia_msgs::execute_cmdResponse &response) {
    return client_.call(request, response);
  }
 private:
  ros::ServiceClient client_;
};

TEST(VisionServer, core_test) {

  provider_vision::VisionServer provider_vision(*nhp);
  std::thread vision_server_thread(RunVisionServer);

  ServiceSubscriber serviceSubscriber("/provider_vision/execute_cmd");

  sonia_msgs::execute_cmdRequest request;
  sonia_msgs::execute_cmdResponse response;


  // Stop a non existing detection task
  request.cmd = request.STOP;
  request.node_name = node_prefix + "INVALID";
  request.filterchain_name = "INVALID";
  request.media_name = "INVALID";

  serviceSubscriber.CallServer(request, response);
  ASSERT_TRUE(response.response.compare("") == 0);

  // Call a media that doesn't exist.
  request.cmd = request.START;
  request.node_name = node_prefix + "Testing";
  request.filterchain_name = "camera_feed";
  request.media_name = "INVALID";

  serviceSubscriber.CallServer(request, response);
  ASSERT_TRUE(response.response.compare("") == 0);

  // Call a filterchain that doesn't exist.
  request.filterchain_name = "INVALID";
  request.media_name = test_dir + "img/test_image.png";
  serviceSubscriber.CallServer(request, response);
  ASSERT_TRUE(response.response.compare("") == 0);

  // Create an execution that exist.
  request.filterchain_name = "camera_feed";
  serviceSubscriber.CallServer(request, response);
  ASSERT_TRUE(response.response.compare(node_prefix + "Testing") == 0);

  // Close the execution.
  request.cmd = request.STOP;
  serviceSubscriber.CallServer(request, response);
  ASSERT_TRUE(response.response.compare("") == 0);

  // Close the execution (again)
  serviceSubscriber.CallServer(request, response);
  ASSERT_TRUE(response.response.compare("") == 0);

  stop_thread = true;
  vision_server_thread.join();

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
