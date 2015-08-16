/**
 * \file	CamCameraDC1394.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_CAMERA_DC1394_H_
#define VISION_SERVER_CAM_CAMERA_DC1394_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <mutex>
#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <dc1394/dc1394.h>
#include <lib_atlas/sys/timer.h>
#include "utils/camera_id.h"
#include "utils/yuv.h"
#include "media/media.h"
#include "media/cam_driver.h"
#include "media/camera.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

class CAMCameraDC1394 : public Camera {
 public:
  static const int DMA_BUFFER = 4;

  const std::string CAM_TAG;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  CAMCameraDC1394(dc1394camera_t *camera, CameraID id);

  ~CAMCameraDC1394();

  //==========================================================================
  // P U B L I C   M E T H O D S

  // MediaCAM override
  bool Open() override;

  bool Close() override;

  bool Start() override;

  bool Stop() override;

  bool NextImage(cv::Mat &img) override;

  bool IsRealCamera() const override;

  bool SetFeature(FEATURE feat, float value) override;

  float GetFeature(FEATURE feat) override;

  // Sets to different streaming format.
  bool SetFormat7();

  bool SetNormalFormat();

  bool SetCameraParams();

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  virtual uint64_t GetGUID() const;

  CameraID GetCamID() const;

  std::string GetModel() const;

  void SetCamID(CameraID cam_id);

  void SetModel(std::string model);

  bool HasArtificialFramerate() override;

  double GetAcquistionTimerValue();

  // SHOULD BE USE ONLY BY DRIVER WITH CAUTION
  dc1394camera_t *GetCameraPtr();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  // float to enum
  uint32_t ConvertFramerate(float val);

  // enum to float
  float ConvertFramerate(uint32_t val);

  // Corrects the fisheye effect.
  void UndistordImage(cv::Mat &in, cv::Mat &out);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  bool _is_transmitting;

  std::string _model;

  mutable std::mutex _cam_access;

  mutable std::mutex _timer_acces;

  dc1394camera_t *_dc1394_camera;

  dc1394video_mode_t _video_mode;

  atlas::MilliTimer _acquisition_timer;

  // TODO:
  // Maybe not necessary? only performance issue.. Lot of access for
  // getting the matrices of CameraUndistordMatrices, so copy the values here
  // at construction to prevent all those access all the time...
  cv::Mat _camera_matrix;

  cv::Mat _distortion_matrix;

  bool _undistortion_is_enable;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline uint64_t CAMCameraDC1394::GetGUID() const { return _id.GetGUID(); }

//------------------------------------------------------------------------------
//
inline CameraID CAMCameraDC1394::GetCamID() const { return _id; }

//------------------------------------------------------------------------------
//
inline std::string CAMCameraDC1394::GetModel() const { return _model; }

//------------------------------------------------------------------------------
//
inline void CAMCameraDC1394::SetCamID(CameraID cam_id) { _id = cam_id; }

//------------------------------------------------------------------------------
//
inline void CAMCameraDC1394::SetModel(std::string model) { _model = model; }

//------------------------------------------------------------------------------
//
inline bool CAMCameraDC1394::HasArtificialFramerate() { return false; }

//------------------------------------------------------------------------------
//
inline double CAMCameraDC1394::GetAcquistionTimerValue() {
  _timer_acces.lock();
  _acquisition_timer.sleep(3);
  auto timer = _acquisition_timer.time();
  _timer_acces.unlock();
  return timer;
};

//------------------------------------------------------------------------------
//
inline dc1394camera_t *CAMCameraDC1394::GetCameraPtr() {
  return _dc1394_camera;
}

//------------------------------------------------------------------------------
//
inline void CAMCameraDC1394::UndistordImage(cv::Mat &in, cv::Mat &out) {
  if (_undistortion_is_enable) {
    cv::undistort(in, out, _camera_matrix, _distortion_matrix);
  } else {
    in.copyTo(out);
  }
}

//------------------------------------------------------------------------------
//
inline bool CAMCameraDC1394::IsRealCamera() const { return true; }

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_CAMERA_DC1394_H_
