/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévost <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include "provider_vision/media/camera/dc1394_camera.h"
#include <ros/ros.h>
#include <string>

namespace provider_vision {

const char *DC1394Camera::CAM_TAG = "[DC1394 Camera]";

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
bool DC1394Camera::Open() {
  if (IsOpened()) {
    // The goal is to open camera. It is already open, so goal is obtain.
    ROS_WARN_NAMED(CAM_TAG, "The media is already started");
    return true;
  }

  dc1394error_t err;

  std::lock_guard<std::mutex> guard(cam_access_);

  bool opening_result = SetFormat7();
  err = dc1394_capture_setup(dc1394_camera_, DMA_BUFFER,
                             DC1394_CAPTURE_FLAGS_DEFAULT);

  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Error on opening camera %s : %s",
                    media_name_.c_str(), dc1394_error_get_string(err));
    opening_result = false;
  }

  opening_result ? status_ = Status::OPEN : status_ = Status::ERROR;
  return opening_result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::Close() {
  if (IsClosed()) {
    // The goal is to close camera. It is already close, so goal is obtain.
    ROS_WARN_NAMED(CAM_TAG, "The media is already closed");
    return true;
  }

  std::lock_guard<std::mutex> guard(cam_access_);

  bool close_result = true;
  if (status_ == Status::STREAMING) {
    close_result = StopStreaming();
  }

  dc1394error_t error = dc1394_capture_stop(dc1394_camera_);
  if (error != DC1394_SUCCESS) {
    close_result = false;
    ROS_ERROR_NAMED(CAM_TAG, "The media could not be closed: %s",
                    dc1394_error_get_string(error));
  }

  close_result ? status_ = Status::CLOSE : status_ = Status::ERROR;
  return close_result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetStreamingModeOn() {
  cam_access_.lock();
  dc1394error_t error =
      dc1394_video_set_transmission(dc1394_camera_, DC1394_ON);
  cam_access_.unlock();

  bool result = error == DC1394_SUCCESS;
  if (!result) {
    ROS_ERROR_NAMED(CAM_TAG, "The media could not be started: %s",
                    dc1394_error_get_string(error));
  }
  result ? status_ = Status::STREAMING : status_ = Status::ERROR;
  return result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetStreamingModeOff() {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error =
      dc1394_video_set_transmission(dc1394_camera_, DC1394_OFF);

  bool result = (error == DC1394_SUCCESS);

  if (!result) {
    ROS_ERROR_NAMED(CAM_TAG, "The media could not be stopped: %s",
                    dc1394_error_get_string(error));
  }

  result ? status_ = Status::OPEN : status_ = Status::ERROR;
  // Here stopping timer just in case... Should already be closed....
  std::lock_guard<std::mutex> guard2(timer_access_);
  try {
    acquisition_timer_.Pause();
    acquisition_timer_.Reset();
  } catch (std::exception &e) {
    ROS_ERROR_NAMED(CAM_TAG, "Exception on timer handling %s", e.what());
  }
  return result;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::NextImage(cv::Mat &img) {
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
    ROS_ERROR_NAMED(CAM_TAG, "Error on image acquisition %s",
                    dc1394_error_get_string(error));
    return false;
  }

  try {
    cv::Mat tmp =
        cv::Mat(frame->size[1], frame->size[0], CV_8UC2, frame->image);
    if (width_ + x_offset_ > frame->size[1] | width_ == 0)
      width_ = frame->size[1];
    if (height_ + y_offset_ > frame->size[0] | width_ == 0)
      height_ = frame->size[0];
    if (x_offset_ > frame->size[1]) x_offset_ = 0;
    if (y_offset_ > frame->size[0]) y_offset_ = 0;
    cv::Mat croppedImage = tmp(cv::Rect(x_offset_, y_offset_, width_, height_));
    cv::cvtColor(tmp, tmp, CV_YUV2BGR_Y422);
    undistord_matrix_.CorrectInmage(croppedImage, img);
  } catch (cv::Exception &e) {
    status_ = Status::ERROR;
    ROS_ERROR_NAMED(CAM_TAG, "Error on OpenCV image transformation %s",
                    e.what());
    return false;
  }

  // Clean, prepare for new frame.
  cam_access_.lock();
  error = dc1394_capture_enqueue(dc1394_camera_, frame);
  cam_access_.unlock();
  if (error != DC1394_SUCCESS) {
    status_ = Status::ERROR;
    ROS_ERROR_NAMED(CAM_TAG, "Error on image acquisition %s",
                    dc1394_error_get_string(error));
    return false;
  }

  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "The image is empty, there is a problem with the media");
    return false;
  }

  //++calibrate_count_;

  //  if (calibrate_count_ == 10) {
  //    calibrator_.Calibrate(this, img);
  //    calibrate_count_ = 0;
  //  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetGainMode(bool &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  dc1394feature_mode_t mode;
  error = dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_GAIN, &mode);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the gain automatic value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  (mode == DC1394_FEATURE_MODE_AUTO) ? value = FeatureMode::AUTO
                                     : value = FeatureMode::MANUAL;
  // One push makes an auto calib than switch to manual, so will consider it as
  // manual.
  // Mode manual or one push
  return true;

  return false;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetGainValue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value_tmp;
  error =
      dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_GAIN, &value_tmp);
  value = static_cast<double>(value_tmp);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the gain value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetGammaValue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;

  uint32_t value_tmp;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_GAMMA,
                                   &value_tmp);
  value = static_cast<double>(value);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the gamma value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetExposureValue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value_tmp;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_EXPOSURE,
                                   &value_tmp);
  value = static_cast<double>(value_tmp);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the exposure value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetSaturationValue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value_tmp;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_SATURATION,
                                   &value_tmp);
  value = static_cast<double>(value_tmp);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the saturation value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetGainMode(bool mode) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;

  dc1394feature_mode_t mode_dc = (mode == FeatureMode::AUTO)
                                     ? DC1394_FEATURE_MODE_AUTO
                                     : DC1394_FEATURE_MODE_MANUAL;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_GAIN, mode_dc);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the automatic gain %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetGainValue(double value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAIN, value);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the gain value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetGammaValue(double value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_GAMMA, value);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the gamma value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetExposureValue(double value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error =
      dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_EXPOSURE, value);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the exposure value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetSaturationValue(double value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_SATURATION,
                                   value);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the saturation value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetShutterValue(double value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error =
      dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_SHUTTER, value);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the shutter value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetShutterMode(bool mode) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;

  dc1394feature_mode_t mode_dc = (mode == FeatureMode::AUTO)
                                     ? DC1394_FEATURE_MODE_AUTO
                                     : DC1394_FEATURE_MODE_MANUAL;
  error =
      dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER, mode_dc);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the shutter automatic value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetFrameRateValue(double value) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  error = dc1394_feature_set_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   ConvertFramerateToEnum((int32_t)value));
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the framerate value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  // Here testing to make sure it works.
  uint32_t test;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   &test);
  ROS_INFO_NAMED(CAM_TAG, "Setting the framerate to: %f, got %d", value, test);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the framerate value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetShutterValue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value_tmp;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_SHUTTER,
                                   &value_tmp);
  value = static_cast<double>(value_tmp);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the shutter value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetFrameRateValue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t value_tmp;
  error = dc1394_feature_get_value(dc1394_camera_, DC1394_FEATURE_FRAME_RATE,
                                   &value_tmp);
  value = static_cast<double>(value_tmp);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the framerate value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetWhiteBalanceMode(bool mode) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;

  dc1394feature_mode_t mode_dc = (mode == FeatureMode::AUTO)
                                     ? DC1394_FEATURE_MODE_AUTO
                                     : DC1394_FEATURE_MODE_MANUAL;
  error = dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_WHITE_BALANCE,
                                  mode_dc);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot Set the white balance automatic value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetWhiteBalanceMode(bool &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  dc1394feature_mode_t mode;
  error = dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_WHITE_BALANCE,
                                  &mode);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the white balance automatic value %s",
                    dc1394_error_get_string(error));
    return false;
  }

  (mode == DC1394_FEATURE_MODE_AUTO) ? value = FeatureMode::AUTO
                                     : value = FeatureMode::MANUAL;
  // One push makes an auto calib than switch to manual, so will consider it as
  // manual.
  // Mode manual or one push
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetWhiteBalanceRed(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  error = dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
  value = static_cast<double>(red);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the white balance red value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetWhiteBalanceRedValue(double value) {
  dc1394error_t error;
  double blue;

  if (!GetWhiteBalanceBlue(blue)) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Cannot get the white balance blue value, "
                    "so cannot set the whitebalance red ");
    return false;
  }
  std::lock_guard<std::mutex> guard(cam_access_);
  error = dc1394_feature_whitebalance_set_value(dc1394_camera_,
                                                static_cast<uint32_t>(blue),
                                                static_cast<uint32_t>(value));

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the whitebalance red value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetWhiteBalanceGreenValue(double value) {
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature WhiteBalanceGreen is not available on DC1394 "
                 "cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetWhiteBalanceGreen(double &value) const {
  ROS_WARN_NAMED(
      CAM_TAG,
      "The feature WhiteBalanceGreen is not available on DC1394 cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetWhiteBalanceBlueValue(double value) {
  dc1394error_t error;
  double red;

  if (!GetWhiteBalanceBlue(red)) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Cannot get the white balance red value, "
                    "so cannot set the whitebalance blue");
    return false;
  }

  std::lock_guard<std::mutex> guard(cam_access_);

  error = dc1394_feature_whitebalance_set_value(
      dc1394_camera_, static_cast<uint32_t>(value), static_cast<uint32_t>(red));
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the whitebalance blue value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetWhiteBalanceBlue(double &value) const {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;
  uint32_t blue, red;
  error = dc1394_feature_whitebalance_get_value(dc1394_camera_, &blue, &red);
  value = static_cast<double>(blue);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the white balance blue value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
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
bool DC1394Camera::GetShutterMode(bool &value) const {
  dc1394error_t error;
  dc1394feature_mode_t mode;
  error =
      dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_SHUTTER, &mode);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the shutter value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  // One push makes an auto calib than switch to manual, so will consider it as
  // manual.
  // Mode manual or one push
  mode == DC1394_FEATURE_MODE_AUTO ? value = FeatureMode::AUTO
                                   : value = FeatureMode::MANUAL;
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::GetExposureMode(bool &value) const {
  dc1394error_t error;
  dc1394feature_mode_t mode;
  error =
      dc1394_feature_get_mode(dc1394_camera_, DC1394_FEATURE_EXPOSURE, &mode);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get the exposure automatic value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  // One push makes an auto calib than switch to manual, so will consider it as
  // manual.
  // Mode manual or one push
  mode == DC1394_FEATURE_MODE_AUTO ? value = FeatureMode::AUTO
                                   : value = FeatureMode::MANUAL;
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetExposureMode(bool mode) {
  std::lock_guard<std::mutex> guard(cam_access_);
  dc1394error_t error;

  dc1394feature_mode_t mode_dc = (mode == FeatureMode::AUTO)
                                     ? DC1394_FEATURE_MODE_AUTO
                                     : DC1394_FEATURE_MODE_MANUAL;
  error =
      dc1394_feature_set_mode(dc1394_camera_, DC1394_FEATURE_EXPOSURE, mode_dc);

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Cannot set the exposure automatic value %s",
                    dc1394_error_get_string(error));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
float DC1394Camera::ConvertFramerateToFloat(uint32_t val) const {
  float return_val = INVALID_FLOAT;
  if (val == DC1394_FRAMERATE_15) {
    return_val = 15;
  } else if (val == DC1394_FRAMERATE_30) {
    return_val = 30;
  } else if (val == DC1394_FRAMERATE_7_5) {
    return_val = 7.5;
  }
  return return_val;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetFormat7() {
  dc1394error_t err;
  // Sets the ISO speed to maximum.
  if (dc1394_camera_->bmode_capable) {
    err = dc1394_video_set_operation_mode(dc1394_camera_,
                                          DC1394_OPERATION_MODE_1394B);
    if (err != DC1394_SUCCESS) {
      ROS_ERROR_NAMED(CAM_TAG, "Error while setting b mode on camera %s",
                      dc1394_error_get_string(err));
      return false;
    }

    // Setting it to 800 creates issues (surprisingly...)
    err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
    if (err != DC1394_SUCCESS) {
      ROS_ERROR_NAMED(CAM_TAG, "Error while setting iso mode on camera %s",
                      dc1394_error_get_string(err));
      return false;
    }

  } else {
    err = dc1394_video_set_operation_mode(dc1394_camera_,
                                          DC1394_OPERATION_MODE_LEGACY);
    if (err != DC1394_SUCCESS) {
      ROS_ERROR_NAMED(CAM_TAG, "Error while setting legaccy mode on camera %s",
                      dc1394_error_get_string(err));
      return false;
    }

    err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
    if (err != DC1394_SUCCESS) {
      ROS_ERROR_NAMED(CAM_TAG, "Error while setting b mode on camera %s",
                      dc1394_error_get_string(err));
      return false;
    }
  }

  // Sets the mode to format 7
  err = dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Error while setting video mode on camera %s",
                    dc1394_error_get_string(err));
    return false;
  }

  // By default, for Guppy
  uint w, h;
  err = dc1394_format7_get_max_image_size(dc1394_camera_,
                                          DC1394_VIDEO_MODE_FORMAT7_0, &w, &h);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Error while getting max image size on camera %s",
                    dc1394_error_get_string(err));
    return false;
  }

  err = dc1394_format7_set_roi(dc1394_camera_, DC1394_VIDEO_MODE_FORMAT7_0,
                               DC1394_COLOR_CODING_YUV422,
                               DC1394_USE_MAX_AVAIL,  // use max packet size
                               0, 0,                  // left, top
                               w, h);                 // width, height

  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Error while setting roi on camera %s",
                    dc1394_error_get_string(err));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetNormalFormat() {
  dc1394error_t err;

  // ISO SPEED
  err = dc1394_video_set_iso_speed(dc1394_camera_, DC1394_ISO_SPEED_400);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Could not set the ISO speed: %s",
                    dc1394_error_get_string(err));
    return false;
  }

  // VIDEO MODE
  if (std::string(dc1394_camera_->vendor).compare(std::string("AVT")) == 0) {
    err =
        dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_800x600_YUV422);
  } else {
    err =
        dc1394_video_set_mode(dc1394_camera_, DC1394_VIDEO_MODE_640x480_YUV422);
  }
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Could not set the video mode: %s",
                    dc1394_error_get_string(err));
    return false;
  }

  // FRAMERATE
  err = dc1394_video_set_framerate(dc1394_camera_, DC1394_FRAMERATE_15);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Could not set the framerate: %s",
                    dc1394_error_get_string(err));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool DC1394Camera::SetCameraParams() {
  bool all_set = true;

  all_set &= SetFeature(Feature::EXPOSURE_AUTO, exposure_auto_);
  atlas::MilliTimer::Sleep(100);
  if (exposure_auto_) {
    all_set &= SetFeature(Feature::EXPOSURE_VALUE, exposure_);
    atlas::MilliTimer::Sleep(100);
  }
  all_set &= SetFeature(Feature::GAIN_AUTO, gain_auto_);
  atlas::MilliTimer::Sleep(100);
  if (!gain_auto_) {
    all_set &= SetFeature(Feature::GAIN_VALUE, gain_);
    atlas::MilliTimer::Sleep(100);
  }
  all_set &= SetFeature(Feature::SHUTTER_AUTO, shutter_auto_);
  atlas::MilliTimer::Sleep(100);
  if (!shutter_auto_) {
    all_set &= SetFeature(Feature::SHUTTER_VALUE, shutter_);
    atlas::MilliTimer::Sleep(100);
  }
  all_set &= SetFeature(Feature::WHITE_BALANCE_AUTO, white_balance_auto_);
  atlas::MilliTimer::Sleep(100);
  if (!white_balance_auto_) {
    all_set &=
        SetFeature(Feature::WHITE_BALANCE_BLUE_VALUE, white_balance_blue_);
    atlas::MilliTimer::Sleep(100);
    all_set &= SetFeature(Feature::WHITE_BALANCE_RED_VALUE, white_balance_red_);
    atlas::MilliTimer::Sleep(100);
  }
  return all_set;
}

}  // namespace provider_vision
