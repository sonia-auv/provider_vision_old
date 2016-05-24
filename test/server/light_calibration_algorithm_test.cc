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
#include <dc1394/dc1394.h>

class FirewireCamera {
 public:
  FirewireCamera() : camera_(nullptr) {}
  ~FirewireCamera() { dc1394_camera_free(camera_); }

  void SetCamera(dc1394camera_t *camera) { camera_ = camera; }

  bool StartCamera();
  bool GetNextImage(cv::Mat &image);
  bool StopCamera();

  void SetGain(uint32_t value);
  void SetExposure(uint32_t value);
  void SetGamma(uint32_t value);
  void SetSaturation(uint32_t value);

  uint32_t GetGain();
  uint32_t GetExposure();
  uint32_t GetGamma();
  uint32_t GetSaturation();

 private:
  void SetFeature(uint32_t value, dc1394feature_t feature);
  uint32_t GetFeature(dc1394feature_t feature);
  dc1394camera_t *camera_;
};

dc1394_t *context_;

bool init(FirewireCamera &firewire_camera);
bool close();

TEST(CalibrationAlgorithm, core_test) {
  FirewireCamera cam;
  init(cam);
  if (!cam.StartCamera()) return;
  for (int i = 0; i < 4; i++) {
    cv::Mat tmp;
    cam.GetNextImage(tmp);
    cv::imshow("SAS", tmp);
    cv::waitKey(-1);
  }
  cam.StopCamera();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

bool init(FirewireCamera &firewire_camera) {
  context_ = dc1394_new();

  dc1394camera_list_t *list;
  dc1394error_t err;

  context_ = dc1394_new();
  if (!context_) return false;
  err = dc1394_camera_enumerate(context_, &list);
  DC1394_ERR_RTN(err, "Failed to enumerate cameras");

  if (list->num == 0) {
    dc1394_log_error("No cameras found");
    return false;
  }

  dc1394camera_t *camera = dc1394_camera_new(context_, list->ids[0].guid);
  if (!camera) {
    dc1394_log_error("Failed to initialize camera with guid %llx",
                     list->ids[0].guid);
    return false;
  }
  dc1394_camera_free_list(list);

  printf("Using camera with GUID %d\n", camera->guid);

  firewire_camera.SetCamera(camera);

  return true;
}

//==============================================================================
//                          FirewireCamera
//==============================================================================
bool FirewireCamera::StartCamera() {
  dc1394error_t err;

  err = dc1394_video_set_iso_speed(camera_, DC1394_ISO_SPEED_400);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  err = dc1394_video_set_mode(camera_, DC1394_VIDEO_MODE_640x480_YUV422);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  err = dc1394_video_set_framerate(camera_, DC1394_FRAMERATE_15);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  err = dc1394_capture_setup(camera_, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  err = dc1394_video_set_transmission(camera_, DC1394_ON);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  return true;
}

bool FirewireCamera::GetNextImage(cv::Mat &img) {
  dc1394video_frame_t *frame = nullptr;
  dc1394error_t error;

  error = dc1394_capture_dequeue(camera_, DC1394_CAPTURE_POLICY_WAIT, &frame);

  if (error != DC1394_SUCCESS || frame == nullptr) {
    return false;
  }

  try {
    cv::Mat tmp =
        cv::Mat(frame->size[1], frame->size[0], CV_8UC2, frame->image);
    cv::cvtColor(tmp, img, CV_YUV2BGR_Y422);
  } catch (cv::Exception &e) {
    return false;
  }
  // Clean, prepare for new frame.
  error = dc1394_capture_enqueue(camera_, frame);

  if (error != DC1394_SUCCESS || img.empty() || img.size().height == 0 ||
      img.size().height == 0) {
    return false;
  }
  return true;
}

bool FirewireCamera::StopCamera() {
  dc1394error_t err = dc1394_video_set_transmission(camera_, DC1394_OFF);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  err = dc1394_capture_stop(camera_);
  if (err != DC1394_SUCCESS) {
    return false;
  }
  return true;
}

void FirewireCamera::SetGain(uint32_t value) {
  SetFeature(value, DC1394_FEATURE_GAIN);
}
void FirewireCamera::SetExposure(uint32_t value) {
  SetFeature(value, DC1394_FEATURE_EXPOSURE);
}
void FirewireCamera::SetGamma(uint32_t value) {
  SetFeature(value, DC1394_FEATURE_GAMMA);
}
void FirewireCamera::SetSaturation(uint32_t value) {
  SetFeature(value, DC1394_FEATURE_SATURATION);
}
void FirewireCamera::SetFeature(uint32_t value, dc1394feature_t feature) {
  dc1394error_t error;
  error = dc1394_feature_set_value(camera_, feature, value);
  if (error != DC1394_SUCCESS) {
    std::cout << "ERROR setting a feature: " << feature << "To value: " << value
              << std::endl;
  }
}
uint32_t FirewireCamera::GetGain() { return GetFeature(DC1394_FEATURE_GAIN); }
uint32_t FirewireCamera::GetExposure() {
  return GetFeature(DC1394_FEATURE_EXPOSURE);
}
uint32_t FirewireCamera::GetGamma() { return GetFeature(DC1394_FEATURE_GAMMA); }
uint32_t FirewireCamera::GetSaturation() {
  return GetFeature(DC1394_FEATURE_SATURATION);
}
uint32_t FirewireCamera::GetFeature(dc1394feature_t feature) {
  dc1394error_t error;
  uint32_t value;
  error = dc1394_feature_get_value(camera_, feature, &value);
  if (error != DC1394_SUCCESS) {
    std::cout << "ERROR getting a feature: " << feature << "To value: " << value
              << std::endl;
    return -1;
  }
  return value;
}
