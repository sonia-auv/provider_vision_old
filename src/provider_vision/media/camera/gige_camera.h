/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
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

#ifndef PROVIDER_VISION_GIGE_CAMERA_H
#define PROVIDER_VISION_GIGE_CAMERA_H

#include <GenApi/GenApi.h>
#include <gevapi.h>
#include <lib_atlas/sys/timer.h>
#include <sensor_msgs/image_encodings.h>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include "provider_vision/media/camera/base_camera.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/context/base_context.h"

namespace provider_vision {

class GigeCamera : public BaseCamera {
 public:
  static const int DMA_BUFFER = 4;
  static constexpr float FPS = 15;

  static const char *CAM_TAG;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<GigeCamera>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit GigeCamera(const CameraConfiguration &config);

  virtual ~GigeCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  // BaseCamera override
  bool Open() override;

  bool Close() override;

  bool SetStreamingModeOn() override;

  bool SetStreamingModeOff() override;

  bool NextImage(cv::Mat &img) override;

  double GetAcquistionTimerValue() const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  bool SetGainMode(bool value) override;
  bool SetGainValue(double value) override;
  bool GetGainMode(bool &value) const override;
  bool GetGainValue(double &value) const override;

  bool GetGammaValue(double &value) const override;
  bool SetGammaValue(double value) override;

  bool GetExposureValue(double &value) const override;
  bool SetExposureValue(double value) override;
  bool SetExposureMode(bool value) override;
  bool GetExposureMode(bool &value) const override;

  bool GetSaturationValue(double &value) const override;
  bool SetSaturationValue(double value) override;

  bool SetShutterValue(double value) override;
  bool SetShutterMode(bool value) override;
  bool GetShutterMode(bool &value) const override;
  bool GetShutterValue(double &value) const override;

  bool SetFrameRateValue(double value) override;
  bool GetFrameRateValue(double &value) const override;

  bool SetWhiteBalanceMode(bool value) override;
  bool GetWhiteBalanceMode(bool &value) const override;
  bool SetWhiteBalanceRedValue(double value) override;
  bool SetWhiteBalanceGreenValue(double value) override;
  bool SetWhiteBalanceBlueValue(double value) override;
  bool GetWhiteBalanceRed(double &value) const override;
  bool GetWhiteBalanceBlue(double &value) const override;
  bool GetWhiteBalanceGreen(double &value) const override;

  /// Specific GigE vision features.
  /// This will be used internally only, mainly for calibration purpose.
  /// If we wished to use these feature widely, we must define them
  /// on the BaseCamera abstract class.
  bool GetWhiteBalanceRatio(double &value) const override;
  bool SetWhiteBalanceRatio(double value);
  bool SetAutoBrightnessMode(double value);

  bool SetAutoBrightnessTarget(double value) override;
  bool SetAutoBrightnessTargetVariation(double value) override;
  bool GetAutoBrightnessTargetVariation(double &value) const override;
  bool GetAutoBrightnessMode(double &value) const override;
  bool GetAutoBrightnessTarget(double &value) const override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void BalanceWhite(cv::Mat mat);

  bool SetCameraParams();

  std::string GetModel() const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  GEV_CAMERA_HANDLE *GetCameraPtr();

  mutable std::mutex cam_access_;

  mutable std::mutex timer_access_;

  GEV_CAMERA_HANDLE gige_camera_;

  atlas::MilliTimer acquisition_timer_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline std::string GigeCamera::GetModel() const {
  if (gige_camera_) {
    GEV_CAMERA_INFO *camera_info = GevGetCameraInfo(gige_camera_);
    return camera_info->model;
  }
  ROS_ERROR_NAMED(CAM_TAG, "Could not retreive camera model");
  return "";
}

//------------------------------------------------------------------------------
//
inline double GigeCamera::GetAcquistionTimerValue() const {
  timer_access_.lock();
  double timer = acquisition_timer_.Time();
  timer_access_.unlock();
  return timer;
}

//------------------------------------------------------------------------------
//
inline GEV_CAMERA_HANDLE *GigeCamera::GetCameraPtr() { return &gige_camera_; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_GIGE_CAMERA_H
