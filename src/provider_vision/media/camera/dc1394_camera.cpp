/**
 * \file	dc1394_camera.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <string>
#include <ros/ros.h>
#include "provider_vision/media/camera/dc1394_camera.h"

namespace vision_server {

const std::string DC1394Camera::CAM_TAG = "[DC1394 Camera]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DC1394Camera::DC1394Camera(dc1394camera_t *camera,
                           const CameraConfiguration &config)
    : BaseCamera(config),
      dc1394_camera_(camera),
      video_mode_(DC1394_VIDEO_MODE_800x600_YUV422) {}

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

  SetFormat7();
  err = dc1394_capture_setup(dc1394_camera_, DMA_BUFFER,
                             DC1394_CAPTURE_FLAGS_DEFAULT);
  if (err != DC1394_SUCCESS) {
    throw std::runtime_error(dc1394_error_get_string(err));
  }
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
  SetCameraParams();

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

  std::lock_guard<std::mutex> guard(cam_access_);

  timer_access_.lock();
  acquisition_timer_.Sleep(3);
  acquisition_timer_.Start();
  timer_access_.unlock();

  error = dc1394_capture_dequeue(dc1394_camera_, DC1394_CAPTURE_POLICY_WAIT,
                                 &frame);

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
  error = dc1394_capture_enqueue(dc1394_camera_, frame);
  if (error != DC1394_SUCCESS) {
    status_ = Status::ERROR;
    throw std::runtime_error("The media is not accessible");
  }
  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    throw std::runtime_error(
        "The image is empty, there is a problem with the media");
  }
}

//------------------------------------------------------------------------------
//
void DC1394Camera::SetFeature(const Feature &feat, float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;

  std::stringstream ss;
  ss << std::hex << config_.GetGUID() << " " << GetName();

  switch (feat) {
    case Feature::SHUTTER:
      //        //ROS_INFO_NAMED(ss.str(), "Setting shutter to %f", value);
      error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                       value);
      break;
    case Feature::SHUTTER_AUTO:

      if (value > 0) {
        //          //ROS_INFO_NAMED(ss.str(), "Setting shutter to auto");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        //          //ROS_INFO_NAMED(ss.str(), "Setting shutter to manual");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::GAIN_AUTO:

      if (value > 0) {
        //          //ROS_INFO_NAMED(ss.str(), "Setting gain to auto");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        //          //ROS_INFO_NAMED(ss.str(), "Setting gain to manual");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::GAIN:
      ////ROS_INFO_NAMED(ss.str(), "Setting gain to manual %f", value);
      error =
          dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAIN, value);
      break;
    case Feature::FRAMERATE:
      ////ROS_INFO_NAMED(ss.str(), "Setting framerate to %f", value);
      error =
          dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   ConvertFramerateToEnum(value));
      uint32_t test;
      error = dc1394_feature_get_value(dc1394_camera_,
                                       DC1394_FEATURE_FRAME_RATE, &test);
      std::cout << "settting to : " << value << " got "
                << ConvertFramerateToEnum(test) << std::endl;
      break;
    case Feature::WHITE_BALANCE_AUTO:
      if (value > 0) {
        ////ROS_INFO_NAMED(ss.str(), "Setting auto white balance to auto");
        error = dc1394_feature_set_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ////ROS_INFO_NAMED(ss.str(), "Setting auto white balance to manual");
        error = dc1394_feature_set_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::WHITE_BALANCE_BLUE:
      ////ROS_INFO_NAMED(ss.str(), "Setting blue value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      blue = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(dc1394_camera_, blue, red);
      break;
    case Feature::WHITE_BALANCE_RED:
      ////ROS_INFO_NAMED(ss.str(), "Setting red value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      red = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(dc1394_camera_, blue, red);
      break;
    case Feature::ERROR_FEATURE:
      break;
  }

  if (error != DC1394_SUCCESS) {
    throw std::runtime_error("The media is not accessible");
  }
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetFeature(const Feature &feat) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  dc1394feature_mode_t mode;
  uint32_t value;
  float return_val;

  switch (feat) {
    case Feature::SHUTTER:
      error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                       &value);
      break;
    case Feature::SHUTTER_AUTO:
      error = dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                      &mode);
      if (mode == DC1394_FEATURE_MODE_MANUAL)
        value = 0;
      else if (mode == DC1394_FEATURE_MODE_AUTO)
        value = 1;
      break;
    case Feature::FRAMERATE:
      error = dc1394_feature_get_value(dc1394_camera_,
                                       DC1394_FEATURE_FRAME_RATE, &value);
      return_val = ConvertFramerateToFloat(static_cast<unsigned int>(value));
      value = static_cast<uint32_t>(return_val);
      break;
    case Feature::WHITE_BALANCE_AUTO:
      error = dc1394_feature_get_mode(dc1394_camera_,
                                      DC1394_FEATURE_WHITE_BALANCE, &mode);
      if (mode == DC1394_FEATURE_MODE_MANUAL)
        value = 0;
      else if (mode == DC1394_FEATURE_MODE_AUTO)
        value = 1;
      break;
    case Feature::WHITE_BALANCE_BLUE:
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      value = blue;
      break;
    case Feature::WHITE_BALANCE_RED:
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      value = red;
      break;
    case Feature::ERROR_FEATURE:
    default:
      break;
  }

  if (error != DC1394_SUCCESS) {
    return -1.0f;
  }

  return static_cast<float>(value);
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
    throw std::runtime_error("An error occurenced");
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
    throw std::runtime_error("An error occurenced");
  }

  // Sets the mode to format 7
  err = dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0);
  if (err != DC1394_SUCCESS) {
    // TODO Jérémie St-Jules: Change the exception error.
    throw std::runtime_error("An error occurenced");
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
    throw std::runtime_error("An error occurenced");
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
    SetFeature(Feature::GAIN_AUTO, 0.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::GAIN, 420.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER_AUTO, 0.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER, 32.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_AUTO, 0.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_BLUE, 381.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_RED, 568.0f);
    atlas::MilliTimer::Sleep(100);
  } else {
    SetFeature(Feature::GAIN_AUTO, 0.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::GAIN, 350.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER_AUTO, 0.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::SHUTTER, 500.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_AUTO, 0.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_BLUE, 412.0f);
    atlas::MilliTimer::Sleep(100);
    SetFeature(Feature::WHITE_BALANCE_RED, 511.0f);
    atlas::MilliTimer::Sleep(100);
  }
}

}  // namespace vision_server
