/**
 * \file	base_camera.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	19/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
#define PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_

#include <memory>
#include <mutex>
#include "provider_vision/config.h"
#include "provider_vision/media/cam_undistord_matrices.h"
#include "provider_vision/media/camera/base_media.h"


namespace provider_vision {

/**
 * Every camera (i.e. not video or image) should inherit this instead of
 * Media class.
 * This enables feature and open/close stuff.
 */
class BaseCamera : public BaseMedia {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseCamera>;

  /**
   * When calling with node, so string, please use
   * the enum name, ex. "FRAMERATE" for FEATURE::FRAMERATE
   */
  enum class Feature {
    ERROR_FEATURE,
    SHUTTER_AUTO,
    SHUTTER_MANUAL,
    SHUTTER,
    GAIN_AUTO,
    GAIN_MANUAL,
    GAIN,
    WHITE_BALANCE_AUTO,
    WHITE_BALANCE_MANUAL,
    WHITE_BALANCE_RED,
    WHITE_BALANCE_BLUE,
    FRAMERATE,
    GAMMA,
    EXPOSURE,
    SATURATION
  };

  struct SPid {
    double d_state;       // Last position input
    double i_state;       // Integrator state
    double i_max, i_min;  // Maximum and minimum allowable integrator state
    double i_gain,        // integral gain
        p_gain,           // proportional gain
        d_gain;           // derivative gain
  };

  /**
   * This structure is only used for the calibration
   * It represent all the features of the camera
   * This is going to be compared with the real values
   * of the features to calculate the error
   */
  struct FeatureValues {
    double gain;
    double gamma;
    double exposure;
    double saturation;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseCamera(const CameraConfiguration &configuration);

  virtual ~BaseCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void SetFeature(const Feature &feat, double value = 0);

  virtual double GetFeature(const Feature &feat) const;

  bool HasArtificialFramerate() const override;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  virtual double GetGainValue() const = 0;
  virtual void SetGainAuto() = 0;
  virtual void SetGainManual() = 0;
  virtual void SetGainValue(double value) = 0;

  virtual double GetGammaValue() const = 0;
  virtual void SetGammaValue(double value) = 0;

  virtual double GetExposureValue() const = 0;
  virtual void SetExposureValue(double value) = 0;

  virtual double GetSaturationValue() const = 0;
  virtual void SetSaturationValue(double value) = 0;

  virtual void SetShutterValue(double value) = 0;
  virtual void SetShutterAuto() = 0;
  virtual void SetShutterManual() = 0;
  virtual double GetShutterMode() const = 0;
  virtual double GetShutterValue() const = 0;

  virtual void SetFrameRateValue(double value) = 0;
  virtual double GetFrameRateValue() const = 0;

  virtual void SetWhiteBalanceAuto() = 0;
  virtual void SetWhiteBalanceManual() = 0;
  virtual double GetWhiteBalanceMode() const = 0;
  virtual void SetWhiteBalanceRedValue(double value) = 0;
  virtual void SetWhiteBalanceBlueValue(double value) = 0;
  virtual double GetWhiteBalanceRed() const = 0;
  virtual double GetWhiteBalanceBlue() const = 0;

  void Calibrate(cv::Mat const &img);

  float CalculateMSV(const cv::Mat &img, int nbrRegion);


  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  CameraConfiguration config_;

  FeatureValues current_features_;

  CameraUndistordMatrices undistord_matrix_;

  SPid gamma_pid_, gain_pid_, exposure_pid_, saturation_pid_;

  double gain_lim_, exposure_lim_;

  float msv_lum_, msv_sat_;

  mutable std::mutex msv_acces_;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  double UpdatePID(SPid &pid, double error, double position) ATLAS_NOEXCEPT;
  cv::Mat CalculateLuminanceHistogram(const cv::Mat &img) const;
  cv::Mat CalculateSaturationHistogram(const cv::Mat &img) const;
  float GetCameraMsvLum() const;
  float GetCameraMsvSat() const;

  friend class CameraParametersListenerTest;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool BaseCamera::HasArtificialFramerate() const { return false; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
