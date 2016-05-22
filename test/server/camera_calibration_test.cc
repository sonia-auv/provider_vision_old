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
#include <provider_vision/server/detection_task.h>
#include <provider_vision/server/detection_task_manager.h>
#include <provider_vision/server/filterchain_manager.h>
#include "provider_vision/server/media_manager.h"

ros::NodeHandle *nhp;
const std::string cam_name = "Webcam";

class CameraParametersListenerTest : public atlas::Observer<const cv::Mat &> {
 public:
  explicit CameraParametersListenerTest(provider_vision::MediaStreamer::Ptr const &media, const std::clock_t timer, std::string date) :
      camera_(nullptr),
      file_(nullptr),
      start_(timer),
      is_calibrated_(false),
      msv_prev_(0),
      msv_lum_(0),
      date_(date)
  {
    /// We don't want any member of the media to be dangling, just to be safe
    /// To much mutex anyway, one more....
    media->image_access_.lock();
    camera_ = dynamic_cast<provider_vision::BaseCamera*>(media->media_.get());
    media->image_access_.unlock();
    assert(media != nullptr);
    file_.open("Values_settings_cam.csv_" + date);
    assert(file_.is_open());

  }
  virtual ~CameraParametersListenerTest() {
  };

  virtual void OnSubjectNotify(atlas::Subject<const cv::Mat &> &subject, const cv::Mat &args) override {
  if(!is_calibrated_) {
    msv_lum_ = camera_->GetCameraMsvLum();
    file_ << msv_lum_ << ",";
    file_ << camera_->GetCameraMsvSat() << ",";
    file_ << camera_->GetFeature(provider_vision::BaseCamera::Feature::GAIN) << ",";
    file_ << camera_->GetFeature(provider_vision::BaseCamera::Feature::GAMMA) << ",";
    file_ << camera_->GetFeature(provider_vision::BaseCamera::Feature::EXPOSURE) << ",";
    file_ << camera_->GetFeature(provider_vision::BaseCamera::Feature::SATURATION) << ",";
    file_ << (std::clock() - start_) / (double) CLOCKS_PER_SEC << "\n";

    if ((msv_prev_ - msv_lum_) <= 0.01) {
      msv_prev_ = msv_lum_;
    }
    else {
      is_calibrated_ = true;
      file_.close();
    }
  }
  }

  virtual bool GetStatus() {
    return is_calibrated_;
  }
 private:
  provider_vision::BaseCamera*  camera_;
  std::fstream file_;
  clock_t start_;
  bool is_calibrated_;
  double msv_prev_, msv_lum_;
  std::string date_;
};

TEST(CameraCalibration, CreateGraph) {
  provider_vision::MediaManager mmng(*nhp);

  mmng.OpenMedia(cam_name);
  ASSERT_EQ(mmng.GetMediaStatus(cam_name), provider_vision::BaseMedia::Status::OPEN);

  // We are able to get the media streamer from the newly created media.
  provider_vision::MediaStreamer::Ptr mstreamer = mmng.StartStreamingMedia(cam_name);
  ASSERT_NE(mstreamer.get(), nullptr);

  // The media streamer is directing to the correct media.
  ASSERT_EQ(mstreamer->GetMediaName().compare(cam_name), 0);

  // The medias that has been created is streaming.
  ASSERT_EQ(mstreamer->GetMediaStatus(), provider_vision::BaseMedia::Status::STREAMING);

  //Creating the filterchain to get the camera feed
  provider_vision::FilterchainManager fcmgr;
  auto fc = fcmgr.InstanciateFilterchain("camera_feed");
  provider_vision::DetectionTaskManager dmgr;
  dmgr.StartDetectionTask(mstreamer, fc, "Calibration_data");


  std::string current_date_ = atlas::Timer<>::CurrentDate();

  //Creating CSV file to output the data
  std::fstream csv_file;
  csv_file.open("Values_settings_cam.csv_" + current_date_);
  csv_file << "MSV Sat,MSV Lum,Gain,Gamma,Exposure,Saturation,Time";

  //Creating timer for time stamping data
  std::clock_t start; 
  start = std::clock();


  CameraParametersListenerTest listener(mstreamer, start, current_date_);
  mstreamer->Attach(listener);

  while (!listener.GetStatus()){}

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
