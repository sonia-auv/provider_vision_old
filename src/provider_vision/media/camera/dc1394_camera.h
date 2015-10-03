/**
 * \file	CamCameraDC1394.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_CAMERA_DC1394_H_
#define PROVIDER_VISION_CAM_CAMERA_DC1394_H_

#include <mutex>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <dc1394/dc1394.h>
#include <lib_atlas/sys/timer.h>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/media/camera/base_camera.h"

namespace vision_server {

class DC1394Camera : public BaseCamera {
 public:
  static const int DMA_BUFFER = 4;

  static const std::string CAM_TAG;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DC1394Camera>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit DC1394Camera(dc1394camera_t *camera,
                        const CameraConfiguration &config);

  ~DC1394Camera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  // BaseCamera override
  bool Open() override;

  bool Close() override;

  bool Start() override;

  bool Stop() override;

  bool NextImage(cv::Mat &img) override;

  bool SetFeature(const Feature &feat, float value) override;

  float GetFeature(const Feature &feat) const override;

  // Sets to different streaming format.
  bool SetFormat7();

  bool SetNormalFormat();

  bool SetCameraParams();

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  std::string GetModel() const;

  double GetAcquistionTimerValue() const;

  // SHOULD BE USE ONLY BY DRIVER WITH CAUTION
  dc1394camera_t *GetCameraPtr() const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  // float to enum
  uint32_t ConvertFramerateToEnum(float val) const;

  // enum to float
  float ConvertFramerateToFloat(uint32_t val) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  mutable std::mutex cam_access_;

  mutable std::mutex timer_access_;

  dc1394camera_t *dc1394_camera_;

  dc1394video_mode_t video_mode_;

  atlas::MilliTimer acquisition_timer_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline std::string DC1394Camera::GetModel() const {
  if (dc1394_camera_ != nullptr) {
    throw std::runtime_error("Null camera pointer");
  }
  return dc1394_camera_->model;
}

//------------------------------------------------------------------------------
//
inline double DC1394Camera::GetAcquistionTimerValue() const {
  timer_access_.lock();
  double timer = acquisition_timer_.Time();
  timer_access_.unlock();
  return timer;
};

//------------------------------------------------------------------------------
//
inline dc1394camera_t *DC1394Camera::GetCameraPtr() const {
  return dc1394_camera_;
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_CAMERA_DC1394_H_
