/**
 * \file  camera_calibration.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date  15/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ros/ros.h>
#include <provider_vision/server/detection_task.h>
#include <provider_vision/server/detection_task_manager.h>
#include <provider_vision/server/filterchain_manager.h>
#include <provider_vision/server/vision_server.h>
#include <sonia_msgs/CameraFeatures.h>

ros::NodeHandle *nhp;
const std::string cam_name = "Bottom Guppy";

class CameraParametersListenerTest {
 public:
  CameraParametersListenerTest(ros::NodeHandlePtr nh) : nh_(nh), listener_(), csv_file_("Camera_parameters_" + atlas::Timer::CurrentDate(), std::ofstream::out) {
    if (csv_file_.is_open()) {
      csv_file_ << "MSV Lum,MSV Sat,Gamma,Gain,Saturation,Exposure\n";
      listener_ = nh_->subscribe(
          "/provider_vision/camera/" + cam_name + "_features", 1000,
          &CameraParametersListenerTest::ListenerCallback, this);
    }
  };
  virtual ~CameraParametersListenerTest(){};

 protected:
  void ListenerCallback(const sonia_msgs::CameraFeatures &msg) {
    csv_file_ << msg.luminance_msv << ",";
    csv_file_ << msg.saturation_msv << ",";
    csv_file_ << msg.gamma_value << ",";
    csv_file_ << msg.gain_value << ",";
    csv_file_ << msg.saturation_value << ",";
    csv_file_ << msg.exposure_value << "\n";
  }

 private:
  ros::NodeHandlePtr nh_;
  ros::Subscriber listener_;
  std::ofstream csv_file_;
};

void StartVisionServer() {
  provider_vision::VisionServer server(*nhp);

  provider_vision::MediaManager mmng(*nhp);
  auto media = mmng.StartStreamingMedia(cam_name);

  provider_vision::FilterchainManager fcmgr;
  auto filterchain = fcmgr.InstanciateFilterchain("test_filterchain");

  provider_vision::DetectionTaskManager dmgr;
  // dmgr.StartDetectionTask(media, filterchain, rqst.node_name);
}

TEST(CameraCalibration, CreateGraph) {
  provider_vision::VisionServer server(*nhp);

  provider_vision::MediaManager mmng(*nhp);
  auto media = mmng.StartStreamingMedia(cam_name);

  provider_vision::FilterchainManager fcmgr;
  auto filterchain = fcmgr.InstanciateFilterchain("test_filterchain");

  // We are able to get the media streamer from the newly created media.
  provider_vision::MediaStreamer::Ptr mstreamer =
      mmng.StartStreamingMedia(cam_name);
  ASSERT_NE(mstreamer.get(), nullptr);

  // The media streamer is directing to the correct media.
  ASSERT_EQ(mstreamer->GetMediaName().compare(cam_name), 0);

  // The medias that has been created is streaming.
  ASSERT_EQ(mstreamer->GetMediaStatus(),
            provider_vision::BaseMedia::Status::STREAMING);

  // Creating the filterchain to get the camera feed
  auto fc = fcmgr.InstanciateFilterchain("camera_feed");
  provider_vision::DetectionTaskManager dmgr;
  dmgr.StartDetectionTask(mstreamer, fc, "Calibration_data");

  
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
