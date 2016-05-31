/**
 * \file  camera_calibration.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date  15/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <sonia_msgs/SetCameraFeature.h>
#include <sonia_msgs/GetCameraFeature.h>
#include <lib_atlas/sys/timer.h>

ros::NodeHandle *nhp;

std::vector<std::string> cam_names = {"front_guppy", "bottom_gige"};

const int FEAT_DBL = sonia_msgs::SetCameraFeature::Request::FEATURE_DOUBLE;
const int FEAT_INT = sonia_msgs::SetCameraFeature::Request::FEATURE_INT;
const int FEAT_BOOL = sonia_msgs::SetCameraFeature::Request::FEATURE_BOOL;

std::tuple<std::string, std::string, int32_t> command_to_send[16];
void fillTuple() {
  int i = 0;
  command_to_send[i++] = std::make_tuple("SHUTTER", "100.0", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("SHUTTER_AUTO", "1", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("SHUTTER_AUTO", "0", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("WHITE_BALANCE_AUTO", "1", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("WHITE_BALANCE_AUTO", "0", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("WHITE_BALANCE_RED", "444", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("WHITE_BALANCE_BLUE", "444", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("FRAMERATE_VALUE", "15.0", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("GAIN_AUTO", "1", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("GAIN_AUTO", "0", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("GAIN", "100", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("EXPOSURE_AUTO", "1", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("EXPOSURE_AUTO", "0", FEAT_BOOL);
  command_to_send[i++] = std::make_tuple("EXPOSURE", "100.0", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("SATURATION", "100.0", FEAT_DBL);
  command_to_send[i++] = std::make_tuple("GAMMA", "100.0", FEAT_DBL);
};

class CameraParameterTest {
 public:
  CameraParameterTest(ros::NodeHandle nh) : nh_(nh) {
    set_client = nh_.serviceClient<sonia_msgs::SetCameraFeature>(
        "/provider_vision/set_camera_feature");
    get_client = nh_.serviceClient<sonia_msgs::GetCameraFeature>(
        "/provider_vision/get_camera_feature");
  };
  ~CameraParameterTest(){};

  void SetFeature(const std::string &camera, const std::string &feature,
                  const std::string &value, int type) {
    sonia_msgs::SetCameraFeature rqst;
    rqst.request.camera_name = camera;
    rqst.request.camera_feature = feature;
    rqst.request.feature_value = value;
    rqst.request.feature_type = type;
    set_client.call(rqst);
  }

  void GetFeature(const std::string &camera, const std::string &feature,
                  std::string &value, int &type) {
    sonia_msgs::GetCameraFeature rqst;
    rqst.request.camera_name = camera;
    rqst.request.camera_feature = feature;
    if (get_client.call(rqst)) {
      std::cout << "Response: " << rqst.response.feature_value << " "
                << rqst.response.feature_type << std::endl;
      value = rqst.response.feature_value;
      type = rqst.response.feature_type;
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::ServiceClient set_client;
  ros::ServiceClient get_client;
};

TEST(TestAll, testALL) {
  fillTuple();
  CameraParameterTest cameraParameterTest(*nhp);

  for (auto cam : cam_names) {
    for (auto cmd : command_to_send) {
      std::cout << "==========" << std::endl;
      std::cout << "[ " << cam << " , " << std::get<0>(cmd) << " , "
                << std::get<1>(cmd) << " , " << std::get<2>(cmd) << " ]"
                << std::endl;
      cameraParameterTest.SetFeature(cam, std::get<0>(cmd), std::get<1>(cmd),
                                     std::get<2>(cmd));
      atlas::MilliTimer::Sleep(100);
      std::string value;
      int type;
      cameraParameterTest.GetFeature(cam, std::get<0>(cmd), value, type);
      std::cout << "[ " << cam << " , " << std::get<0>(cmd) << " , " << value
                << " , " << type << " ]" << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision_test");
  nhp = new ros::NodeHandle("provider_vision_test");
  return RUN_ALL_TESTS();
}
