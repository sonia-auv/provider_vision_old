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
#include "media/cam_camera_dc1394.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMCameraDC1394::CAMCameraDC1394(dc1394camera_t *camera, CameraID id)
    : Camera(id),
      _dc1394_camera(camera),
      _is_transmitting(false),
      _video_mode(DC1394_VIDEO_MODE_800x600_YUV422),
      CAM_TAG("[DC1394 Camera]") {
  _status = CLOSE;
  _undistortion_is_enable = _id._camUndistordMatrices.IsCorrectionEnable();

  if (_undistortion_is_enable)
    _id._camUndistordMatrices.GetMatrices(_camera_matrix, _distortion_matrix);
}

//------------------------------------------------------------------------------
//
CAMCameraDC1394::~CAMCameraDC1394() {
  dc1394_camera_free(_dc1394_camera);
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::Open() {
  dc1394error_t err;
  bool init_result = true;

  std::lock_guard<std::mutex> guard(_cam_access);

  if (!SetFormat7()) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while setting the Format 7 %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(err));
    init_result = false;
  }

  if (init_result) {
    err = dc1394_capture_setup(_dc1394_camera, DMA_BUFFER,
                               DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS) {
      ROS_ERROR_NAMED(
          CAM_TAG, "Could not set the DMA buffer size on %s with message: %s ",
          _id.GetFullName(), dc1394_error_get_string(err));
      init_result = false;
    }
  }
  if (init_result) {
    _status = OPEN;
  } else {
    _status = ERROR;
  }

  return init_result;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::Close() {
  std::lock_guard<std::mutex> guard(_cam_access);

  bool close_result = true;
  if (_is_transmitting) close_result = Stop();

  dc1394error_t error = dc1394_capture_stop(_dc1394_camera);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Capture was not stop properly on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    close_result = false;
  }

  close_result == true ? _status = CLOSE : _status = ERROR;

  return close_result;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::Start() {
  _cam_access.lock();
  dc1394error_t error =
      dc1394_video_set_transmission(_dc1394_camera, DC1394_ON);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Transmission could not start on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    _status = ERROR;
    return false;
  }
  _cam_access.unlock();
  SetCameraParams();

  _is_transmitting = true;
  _status = STREAMING;
  return true;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::Stop() {
  std::lock_guard<std::mutex> guard(_cam_access);

  dc1394error_t error =
      dc1394_video_set_transmission(_dc1394_camera, DC1394_OFF);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Transmission could not stop on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    _status = ERROR;
    return false;
  }

  _is_transmitting = false;
  _status = OPEN;
  // Here stopping timer just in case... Should already be closed....
  std::lock_guard<std::mutex> guard2(_timer_acces);
  return true;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::NextImage(cv::Mat &img) {
  dc1394video_frame_t *frame = nullptr;
  dc1394error_t error;

  /// TEST
  // float blue, red;
  // blue = GetFeature(WHITE_BALANCE_BLUE);
  // red = GetFeature(WHITE_BALANCE_RED);
  // std::cout << "Blue Red: " << blue << " " << red << std::endl;
  /// END TEST

  std::lock_guard<std::mutex> guard(_cam_access);

  _timer_acces.lock();
  _acquisition_timer.sleep(3);
  _acquisition_timer.start();
  _timer_acces.unlock();

  error = dc1394_capture_dequeue(_dc1394_camera, DC1394_CAPTURE_POLICY_WAIT,
                                 &frame);

  _timer_acces.lock();
  atlas::MilliTimer::sleep(3);
  _timer_acces.unlock();

  /// Here we take exactly the camera1394 method... it works so... :P
  if (error != DC1394_SUCCESS || frame == nullptr) {
    ROS_ERROR_NAMED(CAM_TAG, "Capture dequeue failed on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    _status = ERROR;
    return false;
  }

  try {
    cv::Mat tmp =
        cv::Mat(frame->size[1], frame->size[0], CV_8UC2, frame->image);
    cv::cvtColor(tmp, tmp, CV_YUV2BGR_Y422);

    UndistordImage(tmp, img);
  } catch (cv::Exception &e) {
    ROS_ERROR_NAMED(
        CAM_TAG,
        "Conversion from DC1394 frame to cv::Mat failed on %s with message: %s",
        _id.GetFullName(), e.what());
    _status = ERROR;
    return false;
  }

  // Clean, prepare for new frame.
  error = dc1394_capture_enqueue(_dc1394_camera, frame);
  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG, "Capture enqueue failed on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    _status = ERROR;
    return false;
  }
  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    ROS_ERROR_NAMED(CAM_TAG, "Error in image getting on %s", _id.GetFullName());
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::SetFeature(FEATURE feat, float value) {
  std::lock_guard<std::mutex> guard(_cam_access);
  dc1394error_t error;
  uint32_t blue, red;
  switch (feat) {
    case SHUTTER:
      ROS_INFO_NAMED(_id.GetName().c_str(), "Setting shutter to %f", value);
      error = dc1394_feature_set_value(_dc1394_camera, DC1394_FEATURE_SHUTTER,
                                       value);
      break;
    case SHUTTER_AUTO:

      if (value > 0) {
        ROS_INFO_NAMED(_id.GetName().c_str(), "Setting shutter to auto");
        error = dc1394_feature_set_mode(_dc1394_camera, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ROS_INFO_NAMED(_id.GetName().c_str(), "Setting shutter to manual");
        error = dc1394_feature_set_mode(_dc1394_camera, DC1394_FEATURE_SHUTTER,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case GAIN_AUTO:

      if (value > 0) {
        ROS_INFO_NAMED(_id.GetName().c_str(), "Setting gain to auto");
        error = dc1394_feature_set_mode(_dc1394_camera, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ROS_INFO_NAMED(_id.GetName().c_str(), "Setting gain to manual");
        error = dc1394_feature_set_mode(_dc1394_camera, DC1394_FEATURE_GAIN,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case GAIN:
      ROS_INFO_NAMED(_id.GetName().c_str(), "Setting gain to manual %f", value);
      error =
          dc1394_feature_set_value(_dc1394_camera, DC1394_FEATURE_GAIN, value);
      break;
    case FRAMERATE:
      ROS_INFO_NAMED(_id.GetName().c_str(), "Setting framerate to %f", value);
      error = dc1394_feature_set_value(
          _dc1394_camera, DC1394_FEATURE_FRAME_RATE, ConvertFramerate(value));
      uint32_t test;
      error = dc1394_feature_get_value(_dc1394_camera,
                                       DC1394_FEATURE_FRAME_RATE, &test);
      printf("settting to : %d got %d\n", ConvertFramerate(value), test);
      break;
    case WHITE_BALANCE_AUTO:
      if (value > 0) {
        ROS_INFO_NAMED(_id.GetName().c_str(),
                       "Setting auto white balance to auto");
        error = dc1394_feature_set_mode(_dc1394_camera,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_AUTO);
      } else {
        ROS_INFO_NAMED(_id.GetName().c_str(),
                       "Setting auto white balance to manual");
        error = dc1394_feature_set_mode(_dc1394_camera,
                                        DC1394_FEATURE_WHITE_BALANCE,
                                        DC1394_FEATURE_MODE_MANUAL);
      }
      break;
    case WHITE_BALANCE_BLUE:
      ROS_INFO_NAMED(_id.GetName().c_str(), "Setting blue value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(_dc1394_camera, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      blue = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(_dc1394_camera, blue, red);
      break;
    case WHITE_BALANCE_RED:
      ROS_INFO_NAMED(_id.GetName().c_str(), "Setting red value to %f", value);
      error =
          dc1394_feature_whitebalance_get_value(_dc1394_camera, &blue, &red);
      if (error != DC1394_SUCCESS) break;
      red = static_cast<uint32_t>(value);
      error = dc1394_feature_whitebalance_set_value(_dc1394_camera, blue, red);
      break;
    case ERROR_FEATURE:
      break;
  }

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while setting parameter on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
float CAMCameraDC1394::GetFeature(FEATURE feat) {
  std::lock_guard<std::mutex> guard(_cam_access);
  dc1394error_t error;
  uint32_t blue, red;
  dc1394feature_mode_t mode;
  uint32_t value;
  switch (feat) {
    case SHUTTER:
      error = dc1394_feature_get_value(_dc1394_camera, DC1394_FEATURE_SHUTTER,
                                       &value);
      break;
    case SHUTTER_AUTO:
      error = dc1394_feature_get_mode(_dc1394_camera, DC1394_FEATURE_SHUTTER,
                                      &mode);
      if (mode == DC1394_FEATURE_MODE_MANUAL)
        value = 0;
      else if (mode == DC1394_FEATURE_MODE_AUTO)
        value = 1;
      break;
    case FRAMERATE:
      error = dc1394_feature_get_value(_dc1394_camera,
                                       DC1394_FEATURE_FRAME_RATE, &value);
      value = ConvertFramerate(value);
      break;
    case WHITE_BALANCE_AUTO:
      error = dc1394_feature_get_mode(_dc1394_camera,
                                      DC1394_FEATURE_WHITE_BALANCE, &mode);
      if (mode == DC1394_FEATURE_MODE_MANUAL)
        value = 0;
      else if (mode == DC1394_FEATURE_MODE_AUTO)
        value = 1;
      break;
    case WHITE_BALANCE_BLUE:
      error =
          dc1394_feature_whitebalance_get_value(_dc1394_camera, &blue, &red);
      value = blue;
      break;
    case WHITE_BALANCE_RED:
      error =
          dc1394_feature_whitebalance_get_value(_dc1394_camera, &blue, &red);
      value = red;
      break;
    case ERROR_FEATURE:
      break;
  }

  if (error != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while setting parameter on %s with message: %s",
                    _id.GetFullName(), dc1394_error_get_string(error));
    return -1.0f;
  }

  return static_cast<float>(value);
}

//------------------------------------------------------------------------------
//
uint32_t CAMCameraDC1394::ConvertFramerate(float val) {
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
float CAMCameraDC1394::ConvertFramerate(uint32_t val) {
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
bool CAMCameraDC1394::SetFormat7() {
  bool init_result = false;
  if (_dc1394_camera == nullptr) return init_result;

  dc1394error_t err;
  // Sets the ISO speed to maximum.
  if (_dc1394_camera->bmode_capable) {
    err = dc1394_video_set_operation_mode(_dc1394_camera,
                                          DC1394_OPERATION_MODE_1394B);
    err = dc1394_video_set_iso_speed(_dc1394_camera, DC1394_ISO_SPEED_400);
  } else {
    err = dc1394_video_set_operation_mode(_dc1394_camera,
                                          DC1394_OPERATION_MODE_LEGACY);
    err = dc1394_video_set_iso_speed(_dc1394_camera, DC1394_ISO_SPEED_400);
  }

  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(
        CAM_TAG,
        " Could not set operating mode or ISO speed.on %s with message: %s",
        _id.GetFullName(),

        dc1394_error_get_string(err));
    return init_result;
  }

  // Sets the mode to format 7
  err = dc1394_video_set_mode(_dc1394_camera, DC1394_VIDEO_MODE_FORMAT7_0);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set video mode to format7on %s with message: %s",
                    _id.GetFullName(),

                    dc1394_error_get_string(err));
    return init_result;
  }

  // Sets the image width and height. Depending if Unibrain or Guppy Pro, we use
  // different size, founded with coriander.
  std::string camName(_dc1394_camera->vendor);
  // By default, for Guppy
  uint w, h;
  dc1394_format7_get_max_image_size(_dc1394_camera, DC1394_VIDEO_MODE_FORMAT7_0,
                                    &w, &h);

  err = dc1394_format7_set_roi(_dc1394_camera, DC1394_VIDEO_MODE_FORMAT7_0,
                               DC1394_COLOR_CODING_YUV422,
                               DC1394_USE_MAX_AVAIL,  // use max packet size
                               0, 0,                  // left, top
                               w, h);                 // width, height

  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(
        CAM_TAG, "Could not set the video size properly on %s with message: %s",
        _id.GetFullName(),

        dc1394_error_get_string(err));
    return init_result;
  }
  init_result = true;
  return init_result;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::SetNormalFormat() {
  if (_dc1394_camera == nullptr) return false;

  dc1394error_t err;

  err = dc1394_video_set_iso_speed(_dc1394_camera, DC1394_ISO_SPEED_400);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set ISO speed to 400 on %s with message: %s",
                    _id.GetFullName(),

                    dc1394_error_get_string(err));
    return false;
  }

  err = dc1394_video_set_mode(_dc1394_camera, DC1394_VIDEO_MODE_800x600_YUV422);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set the video mode on %s with message: %s",
                    _id.GetFullName(),

                    dc1394_error_get_string(err));
    return false;
  }

  err = dc1394_video_set_framerate(_dc1394_camera, DC1394_FRAMERATE_15);
  if (err != DC1394_SUCCESS) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Could not set the framerate on %s with message: %s",
                    _id.GetFullName(),

                    dc1394_error_get_string(err));
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool CAMCameraDC1394::SetCameraParams() {
  if (_dc1394_camera == nullptr) {
    ROS_ERROR_NAMED(CAM_TAG, "Camera is null when setting params");
    return false;
  }
  bool retVal = true;
  // We have a guppy
  if (std::string(_dc1394_camera->vendor).compare(std::string("AVT")) == 0) {
    ROS_INFO_NAMED(CAM_TAG, "Setting parameters for guppy");
    retVal &= SetFeature(GAIN_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(GAIN, 420.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(SHUTTER_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(SHUTTER, 32.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(WHITE_BALANCE_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(WHITE_BALANCE_BLUE, 381.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(WHITE_BALANCE_RED, 568.0f);
    atlas::MilliTimer::sleep(100);
  } else {  // We have a unibrain
    ROS_INFO_NAMED(CAM_TAG, "Setting parameters for unibrain");
    retVal &= SetFeature(GAIN_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(GAIN, 350.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(SHUTTER_AUTO, 0.0f);  // SET TO MANUAL
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(SHUTTER, 500.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(WHITE_BALANCE_AUTO, 0.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(WHITE_BALANCE_BLUE, 412.0f);
    atlas::MilliTimer::sleep(100);
    retVal &= SetFeature(WHITE_BALANCE_RED, 511.0f);
    atlas::MilliTimer::sleep(100);
  }

  return retVal;
}

}  // namespace vision_server
