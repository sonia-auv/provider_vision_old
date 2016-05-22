//
// Created by sonia on 5/4/16.
//

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

  static const std::string CAM_TAG;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<GigeCamera>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit GigeCamera(const CameraConfiguration &config);

  ~GigeCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  // BaseCamera override
  void Open() override;

  void Close() override;

  void SetStreamingModeOn() override;

  void SetStreamingModeOff() override;

  void NextImage(cv::Mat &img) override;

  void SetCameraParams();

  // Set some GigE specific parameters

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  std::string GetModel() const;

  double GetAcquistionTimerValue() const;

  // SHOULD BE USE ONLY BY DRIVER WITH CAUTION
  GEV_CAMERA_HANDLE *GetCameraPtr();

 protected:
  void SetAutoBrightnessMode(int value);
  void SetAutoBrightnessTarget(int value);
  void SetAutoBrightnessTargetVariation(int value);
  void SetExposureAuto();
  void SetExposureManual();
  float GetWhiteBalanceRatio() const;
  void SetWhiteBalanceRatio(float value);

  float GetGainValue() const override;
  void SetGainAuto() override;
  void SetGainManual() override;
  void SetGainValue(float value) override;

  float GetGammaValue() const override;
  void SetGammaValue(float value) override;

  float GetExposureValue() const override;
  void SetExposureValue(float value) override;

  float GetSaturationValue() const override;
  void SetSaturationValue(float value) override;

  void SetShutterValue(float value) override;
  void SetShutterAuto() override;
  void SetShutterManual() override;
  float GetShutterMode() const override;
  float GetShutterValue() const override;

  void SetFrameRateValue(float value) override;
  float GetFrameRateValue() const override;

  void SetWhiteBalanceAuto() override;
  void SetWhiteBalanceManual() override;
  float GetWhiteBalanceMode() const override;
  void SetWhiteBalanceRedValue(float value) override;
  void SetWhiteBalanceBlueValue(float value) override;
  float GetWhiteBalanceRed() const override;
  float GetWhiteBalanceBlue() const override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void balance_white(cv::Mat mat);

  //==========================================================================
  // P R I V A T E   M E M B E R S

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
  if (gige_camera_ != nullptr) {
    throw std::runtime_error("Null camera pointer");
  }
  GEV_CAMERA_INFO *camera_info = GevGetCameraInfo(gige_camera_);
  return camera_info->model;
}

//------------------------------------------------------------------------------
//
inline double GigeCamera::GetAcquistionTimerValue() const {
  timer_access_.lock();
  double timer = acquisition_timer_.Time();
  timer_access_.unlock();
  return timer;
};

//------------------------------------------------------------------------------
//
inline GEV_CAMERA_HANDLE *GigeCamera::GetCameraPtr() { return &gige_camera_; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_GIGE_CAMERA_H
