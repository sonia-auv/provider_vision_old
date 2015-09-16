/**
 * \file	CamCameraDC1394.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <ros/ros.h>
#include "provider_vision/media/camera/dc1394_camera.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
DC1394Camera::DC1394Camera(dc1394camera_t *camera,
                           const CameraConfiguration &config)
    : BaseCamera(config),
      dc1394_camera_(camera),
      video_mode_(DC1394_VIDEO_MODE_800x600_YUV422),
      CAM_TAG("[DC1394 Camera]") {
  status_ = Status::CLOSE;
}

//------------------------------------------------------------------------------
//
DC1394Camera::~DC1394Camera() { dc1394_camera_free(dc1394_camera_); }

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool DC1394Camera::Open() {
  dc1394error_t err;
  bool init_result = true;

  std::lock_guard<std::mutex> guard(cam_access_);

  if (!SetFormat7()) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while setting the Format 7 %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(err));
    init_result = false;
  }

  if (init_result) {
    err = dc1394_capture_setup(dc1394_camera_, DMA_BUFFER,
                               DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS) {
      ROS_ERROR_NAMED(
          CAM_TAG, "Could not set the DMA buffer size on %s with message: %s ",
          config_.GetGUID(), dc1394_error_get_string(err));
      init_result = false;
    }
  }
  if (init_result) {
    status_ = Status::OPEN;
  } else {
    status_ = Status::ERROR;
  }

  return init_result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::Close() {
  std::lock_guard<std::mutex> guard(cam_access_);

  bool close_result = true;
  if (status_ == Status::STREAMING) close_result = Stop();

  dc1394error_t error = dc1394_capture_stop(dc1394_camera_);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Capture was not stop properly on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    close_result = false;
  }

  close_result == true ? status_ = Status::CLOSE : status_ = Status::ERROR;

  return close_result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::Start() {
  cam_access_.lock();
  dc1394error_t error =
      dc1394_video_set_transmission(dc1394_camera_, DC1394_ON);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Transmission could not start on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    status_ = Status::ERROR;
    return false;
  }
  cam_access_.unlock();
  SetCameraParams();

  status_ = Status::STREAMING;
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::Stop() {
  std::lock_guard<std::mutex> guard(cam_access_);

  dc1394error_t error =
      dc1394_video_set_transmission(dc1394_camera_, DC1394_OFF);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Transmission could not stop on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    status_ = Status::ERROR;
    return false;
  }

  status_ = Status::OPEN;
  // Here stopping timer just in case... Should already be closed....
  std::lock_guard<std::mutex> guard2(timer_access_);
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::NextImage(cv::Mat &img) {
  dc1394video_frame_t *frame = nullptr;
  dc1394error_t error;

  std::lock_guard<std::mutex> guard(cam_access_);

  timer_access_.lock();
  acquisition_timer_.sleep(3);
  acquisition_timer_.start();
  timer_access_.unlock();

  error = dc1394_capture_dequeue(dc1394_camera_, DC1394_CAPTURE_POLICY_WAIT,
                                 &frame);

  timer_access_.lock();
  atlas::MilliTimer::sleep(3);
  timer_access_.unlock();

  /// Here we take exactly the camera1394 method... it works so... :P
  if (error != DC1394_SUCCESS || frame == nullptr) {
    ROS_ERROR_NAMED(CAM_TAG, "Capture dequeue failed on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    status_ = Status::ERROR;
    return false;
  }

  try {
    cv::Mat tmp =
        cv::Mat(frame->size[1], frame->size[0], CV_8UC2, frame->image);
    cv::cvtColor(tmp, tmp, CV_YUV2BGR_Y422);
    undistord_matrix_.CorrectInmage(tmp, img));

  } catch (cv::Exception &e) {
    ROS_ERROR_NAMED(
        CAM_TAG,
        "Conversion from DC1394 frame to cv::Mat failed on %s with message: %s",
        config_.GetGUID(), e.what());
    status_ = Status::ERROR;
    return false;
  }

  // Clean, prepare for new frame.
  error = dc1394_capture_enqueue(dc1394_camera_, frame);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Capture enqueue failed on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    status_ = Status::ERROR;
    return false;
  }
  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    ROS_ERROR_NAMED(CAM_TAG, "Error in image getting on %s", config_.GetGUID());
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetFeature(const Feature &feat, float value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;

  std::stringstream ss;
  ss << std::hex << config_.GetGUID() << " " << GetName();

  switch (feat) {
    case Feature::SHUTTER:
      ROS_INFO_NAMED(ss.str(), "Setting shutter to %f", value);
      error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                       value);
      break;
    case Feature::SHUTTER_AUTO:

      if (value > 0) {
        ROS_INFO_NAMED(ss.str(), "Setting shutter to auto");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ROS_INFO_NAMED(ss.str(), "Setting shutter to manual");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::GAIN_AUTO:

      if (value > 0) {
        ROS_INFO_NAMED(ss.str(), "Setting gain to auto");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ROS_INFO_NAMED(ss.str(), "Setting gain to manual");
        error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::GAIN:
      ROS_INFO_NAMED(ss.str(), "Setting gain to manual %f", value);
      error =
          dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAIN, value);
      break;
    case Feature::FRAMERATE:
      ROS_INFO_NAMED(ss.str(), "Setting framerate to %f", value);
      error = dc1394_feature_set_value(
          dc1394_camera_, DC1394_FEATURE_FRAME_RATE, ConvertFramerate(value));
      uint32_t test;
      error = dc1394_feature_get_value(dc1394_camera_,
                                       DC1394_FEATURE_FRAME_RATE, &test);
      printf("settting to : %d got %d\n", ConvertFramerate(value), test);
      break;
    case Feature::WHITE_BALANCE_AUTO:
      if (value > 0) {
        ROS_INFO_NAMED(ss.str(), "Setting auto white balance to auto");
        error = dc1394_feature_set_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ROS_INFO_NAMED(ss.str(), "Setting auto white balance to manual");
        error = dc1394_feature_set_mode(dc1394_camera_,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case Feature::WHITE_BALANCE_BLUE:
      ROS_INFO_NAMED(ss.str(), "Setting blue value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      blue = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(dc1394_camera_, blue, red);
      break;
    case Feature::WHITE_BALANCE_RED:
      ROS_INFO_NAMED(ss.str(), "Setting red value to %f", value);
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
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while setting parameter on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
float DC1394Camera::GetFeature(const Feature &feat) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  dc1394feature_mode_t mode;
  uint32_t value;

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
      value = ConvertFramerate(value);
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
      break;
  }

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while setting parameter on %s with message: %s",
                    config_.GetGUID(), dc1394_error_get_string(error));
    return -1.0f;
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
uint32_t DC1394Camera::ConvertFramerate(float val) {
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
float DC1394Camera::ConvertFramerate(uint32_t val) {
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
bool DC1394Camera::SetFormat7() {
  bool init_result = false;
  if (dc1394_camera_ == nullptr) return init_result;

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
    ROS_ERROR_NAMED(
        CAM_TAG,
        " Could not set operating mode or ISO speed.on %s with message: %s",
        config_.GetGUID(),

        dc1394_error_get_string(err));
    return init_result;
  }

  // Sets the mode to format 7
  err = dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set video mode to format7on %s with message: %s",
                    config_.GetGUID(),

                    dc1394_error_get_string(err));
    return init_result;
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
    ROS_ERROR_NAMED(
        CAM_TAG, "Could not set the video size properly on %s with message: %s",
        config_.GetGUID(),

        dc1394_error_get_string(err));
    return init_result;
  }
  init_result = true;
  return init_result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetNormalFormat() {
  if (dc1394_camera_ == nullptr) return false;

  dc1394error_t err;

  err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set ISO speed to 400 on %s with message: %s",
                    config_.GetGUID(),

                    dc1394_error_get_string(err));
    return false;
  }

  err = dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_800x600_YUV422);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set the video mode on %s with message: %s",
                    config_.GetGUID(),

                    dc1394_error_get_string(err));
    return false;
  }

  err = dc1394_video_set_framerate(dc1394_camera_, DC1394_FRAMERATE_15);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set the framerate on %s with message: %s",
                    config_.GetGUID(),

                    dc1394_error_get_string(err));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetCameraParams() {
  if (dc1394_camera_ == nullptr) {
    ROS_ERROR_NAMED(CAM_TAG, "Camera is null when setting params");
    return false;
  }
  bool retVal = true;
  // We have a guppy
  if (std::string(dc1394_camera_->vendor).compare(std::string("AVT")) == 0) {
    ROS_INFO_NAMED(CAM_TAG, "Setting parameters for guppy");
    retVal &= SetFeature(Feature::GAIN_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::GAIN, 420.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::SHUTTER_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::SHUTTER, 32.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::WHITE_BALANCE_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::WHITE_BALANCE_BLUE, 381.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::WHITE_BALANCE_RED, 568.0f);
    atlas::MilliTimer::sleep(100);
  } else {  // We have a unibrain
    ROS_INFO_NAMED(CAM_TAG, "Setting parameters for unibrain");
    retVal &= SetFeature(Feature::GAIN_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::GAIN, 350.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::SHUTTER_AUTO, 0.0f);  // SET TO MANUAL
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::SHUTTER, 500.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::WHITE_BALANCE_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::WHITE_BALANCE_BLUE, 412.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(Feature::WHITE_BALANCE_RED, 511.0f);
    atlas::MilliTimer::sleep(100);
  }

  return retVal;
}

}  // namespace vision_server
