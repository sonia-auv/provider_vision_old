/**
 * \file	dc1394_camera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/camera/dc1394_camera.h"
#include <ros/ros.h>
#include <string>

namespace provider_vision {

const std::string DC1394Camera::CAM_TAG = "[DC1394 Camera]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DC1394Camera::DC1394Camera(dc1394camera_t *camera,
                           const CameraConfiguration &config)
    : BaseCamera(config), dc1394_camera_(camera), calibrate_count_(0) {}

//------------------------------------------------------------------------------
//
DC1394Camera::~DC1394Camera() { dc1394_camera_free(dc1394_camera_); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DC1394Camera::Open() {
  if (IsOpened()) {
    throw std::logic_error("The media is already started");
  }

  dc1394error_t err;

  std::lock_guard<std::mutex> guard(cam_access_);

  try {
    SetFormat7();
    err = dc1394_capture_setup(dc1394_camera_, DMA_BUFFER,
                               DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS) {
      throw std::runtime_error(dc1394_error_get_string(err));
    }
  } catch (std::exception &e) {
    ROS_ERROR("%s", e.what());
    ROS_ERROR(
        "Issue with format 7 setting... cannot recover, should close"
        "the node and do firewire magic to reset the camera.");
  };

  status_ = Status::OPEN;
}

//------------------------------------------------------------------------------
//
void DC1394Camera::Close() {
  if (!IsOpened()) {
    throw std::logic_error("The media is not started");
  }

  std::lock_guard<std::mutex> guard(cam_access_);

  bool close_result = true;
  if (status_ == Status::STREAMING) {
    StopStreaming();
  }

  dc1394error_t error = dc1394_capture_stop(dc1394_camera_);
  if (error != DC1394_SUCCESS) {
    close_result = false;
  }

  close_result == true ? status_ = Status::CLOSE : status_ = Status::ERROR;
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetStreamingModeOn() {
  cam_access_.lock();
  dc1394error_t error =
      dc1394_video_set_transmission(dc1394_camera_, DC1394_ON);
  if (error != DC1394_SUCCESS) {
    status_ = Status::ERROR;
  }
  cam_access_.unlock();

  status_ = Status::STREAMING;
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetStreamingModeOff() {
  std::lock_guard<std::mutex> guard(cam_access_);

  dc1394error_t error =
      dc1394_video_set_transmission(dc1394_camera_, DC1394_OFF);
  if (error != DC1394_SUCCESS) {
    status_ = Status::ERROR;
    throw std::runtime_error("The media could not be stoped");
  }

  status_ = Status::OPEN;
  // Here stopping timer just in case... Should already be closed....
  std::lock_guard<std::mutex> guard2(timer_access_);
}

//------------------------------------------------------------------------------
//
void DC1394Camera::NextImage(cv::Mat &img) {
  dc1394video_frame_t *frame = nullptr;
  dc1394error_t error;

  timer_access_.lock();
  acquisition_timer_.Sleep(3);
  acquisition_timer_.Start();
  timer_access_.unlock();

  cam_access_.lock();
  error = dc1394_capture_dequeue(dc1394_camera_, DC1394_CAPTURE_POLICY_WAIT,
                                 &frame);
  cam_access_.unlock();
  timer_access_.lock();
  atlas::MilliTimer::Sleep(3);
  timer_access_.unlock();

  /// Here we take exactly the camera1394 method... it works so... :P
  if (error != DC1394_SUCCESS || frame == nullptr) {
    status_ = Status::ERROR;
    throw std::runtime_error("The media is not accessible");
  }

  try {
    cv::Mat tmp =
        cv::Mat(frame->size[1], frame->size[0], CV_8UC2, frame->image);
    cv::cvtColor(tmp, tmp, CV_YUV2BGR_Y422);
    undistord_matrix_.CorrectInmage(tmp, img);

  } catch (cv::Exception &e) {
    status_ = Status::ERROR;
    throw;
  }

  // Clean, prepare for new frame.
  cam_access_.lock();
  error = dc1394_capture_enqueue(dc1394_camera_, frame);
  cam_access_.unlock();
  if (error != DC1394_SUCCESS) {
    status_ = Status::ERROR;
    throw std::runtime_error("The media is not accessible");
  }

  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    throw std::runtime_error(
        "The image is empty, there is a problem with the media");
  }

  if(calibrate_count_ == 10) {
    Calibrate(img);
    calibrate_count_ = 0;
  }
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetGainValue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_GAIN, &value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't get the gain value on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetGammaValue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;

  uint32_t value;
  error =
      dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_GAMMA, &value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't get the gamma value on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetExposureValue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value;
  error =
      dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_EXPOSURE, &value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't get the exposure value on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetSaturationValue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_SATURATION,
                                   &value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't get the saturation value on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetGainAuto() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                  DC1394_FEATURE_MODE_AUTO);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the gain in automatic on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetGainManual() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                  DC1394_FEATURE_MODE_MANUAL);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the Gain in manual on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetGainValue(float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAIN, value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the gain on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetGammaValue(float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAMMA, value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the gamma on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetExposureValue(float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error =
      dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_EXPOSURE, value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the exposure on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetSaturationValue(float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_SATURATION,
                                   value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the saturation on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetShutterValue(float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error =
      dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_SHUTTER, value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the shutter on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetShutterAuto() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                  DC1394_FEATURE_MODE_AUTO);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the shutter in automatic on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetShutterManual() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                  DC1394_FEATURE_MODE_MANUAL);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the shutter in manual on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetFrameRateValue(float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   ConvertFramerateToEnum(value));

  uint32_t test;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   &test);
  std::cout << "settting to : " << value << " got "
            << ConvertFramerateToEnum(test) << std::endl;
  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't set the frame rate on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetShutterValue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value;
  error =
      dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_SHUTTER, &value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("Canno't get the shutter value on DC1394 Camera" +
                             std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetFrameRateValue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   &value);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't get the frame rate value on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetWhiteBalanceAuto() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_WHITE_BALANCE,
                                  DC1394_FEATURE_MODE_AUTO);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the white balance in automatic on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetWhiteBalanceManual() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_WHITE_BALANCE,
                                  DC1394_FEATURE_MODE_MANUAL);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the white balance in manual on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetWhiteBalanceMode() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  dc1394feature_mode_t mode;
  error = dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_WHITE_BALANCE,
                                  &mode);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't get the white balance mode on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }

  if (mode == DC1394_FEATURE_MODE_MANUAL)
    return 0.0f;
  else if (mode == DC1394_FEATURE_MODE_AUTO)
    return 1.0f;
  else
    throw std::runtime_error(
        "Canno't get the white balance mode on DC1394 Camera");
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetWhiteBalanceRed() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  error = dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't get the white balance mode on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(red);
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetWhiteBalanceRedValue(float value) {
  dc1394error_t error;
  try {
    float blue = GetWhiteBalanceBlue();
    std::lock_guard<std::mutex> guard(cam_access_);
    error = dc1394_feature_whitebalance_set_value(dc1394_camera_,
                                                  static_cast<uint32_t>(blue),
                                                  static_cast<uint32_t>(value));
  } catch (const std::runtime_error &e) {
    throw;
  }

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the white balance red values on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

void DC1394Camera::SetWhiteBalanceBlueValue(float value) {
  dc1394error_t error;
  try {
    float red = GetWhiteBalanceBlue();
    std::lock_guard<std::mutex> guard(cam_access_);

    error = dc1394_feature_whitebalance_set_value(dc1394_camera_,
                                                  static_cast<uint32_t>(value),
                                                  static_cast<uint32_t>(red));
  } catch (const std::runtime_error &e) {
    throw;
  }

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't set the white balance red values on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetWhiteBalanceBlue() const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  error = dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
  ;

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't get the white balance blue value on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }

  return static_cast<float>(blue);
}

//------------------------------------------------------------------------------
//
uint32_t DC1394Camera::ConvertFramerateToEnum(float val) const {
  uint32_t return_val = static_cast<uint32_t>(val);
  if (return_val == 15) {
    return_val = DC1394_FRAMERATE_15;
  } else if (return_val == 30) {
    return_val = DC1394_FRAMERATE_30;
  } else if (return_val == 7) {
    return_val = DC1394_FRAMERATE_7_5;
  }
  return return_val;
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetShutterMode() const {
  dc1394error_t error;
  dc1394feature_mode_t mode;
  error =
      dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER, &mode);

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error(
        "Canno't get the white balance blue value on DC1394 Camera" +
        std::to_string(static_cast<int>(error)));
  }

  if (mode == DC1394_FEATURE_MODE_MANUAL)
    return 0.0f;
  else if (mode == DC1394_FEATURE_MODE_AUTO)
    return 1.0f;
  else
    throw std::runtime_error(
        "Canno't get the white balance blue value on DC1394 Camera");
}

//------------------------------------------------------------------------------
//
float DC1394Camera::ConvertFramerateToFloat(uint32_t val) const {
  float return_val = val;
  if (return_val == DC1394_FRAMERATE_15) {
    return_val = 15;
  } else if (return_val == DC1394_FRAMERATE_30) {
    return_val = 30;
  } else if (return_val == DC1394_FRAMERATE_7_5) {
    return_val = 7.5;
  }
  return return_val;
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetFormat7() {
  if (dc1394_camera_ == nullptr) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("Received null pointer");
  }

  dc1394error_t err;
  // Sets the ISO speed to maximum.
  if (dc1394_camera_->bmode_capable) {
    err = dc1394_video_set_operation_mode(dc1394_camera_,
                                          DC1394_OPERATION_MODE_1394B);
    err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
  } else {
    err = dc1394_video_set_operation_mode(dc1394_camera_,
                                          DC1394_OPERATION_MODE_LEGACY);
    err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
  }

  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("Setting iso speed failed");
  }

  // Sets the mode to format 7
  err = dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0);
  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("Setting format 7 failed");
  }

  // Sets the image width and height. Depending if Unibrain or Guppy Pro, we use
  // different size, founded with coriander.
  std::string camName(dc1394_camera_->vendor);
  // By default, for Guppy
  uint w, h;
  dc1394_format7_get_max_image_size(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0,
                                    &w, &h);

  err = dc1394_format7_set_roi(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0,
                               DC1394_COLOR_CODING_YUV422,
                               DC1394_USE_MAX_AVAIL,  // use max packet size
                               0, 0,                  // left, top
                               w, h);                 // width, height

  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("Setting image size and roi failed");
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetNormalFormat() {
  if (dc1394_camera_ == nullptr) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("An error occurenced");
  }

  dc1394error_t err;

  err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("An error occurenced");
  }

  err = dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_800x600_YUV422);
  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("An error occurenced");
  }

  err = dc1394_video_set_framerate(dc1394_camera_, DC1394_FRAMERATE_15);
  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("An error occurenced");
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetCameraParams() {
  if (dc1394_camera_ == nullptr) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("An error occurenced");
  }
  // We have a guppy
  if (std::string(dc1394_camera_->vendor).compare(std::string("AVT")) == 0) {
    SetFeature(Feature::GAIN_MANUAL);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::GAIN, 420.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER_MANUAL);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER, 32.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_MANUAL);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_BLUE, 381.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_RED, 568.0f);
    atlas::MilliTimer::Sleep(100);
  } else {
    SetFeature(Feature::GAIN_MANUAL);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::GAIN, 350.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER_MANUAL);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER, 500.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_MANUAL);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_BLUE, 412.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_RED, 511.0f);
    atlas::MilliTimer::Sleep(100);
  }
}

}  // namespace provider_vision
