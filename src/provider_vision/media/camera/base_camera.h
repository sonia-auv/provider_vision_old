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
#include "provider_vision/media/cam_undistord_matrices.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/utils/config.h"

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
    double dState;      // Last position input
    double iState;      // Integrator state
    double iMax, iMin;  // Maximum and minimum allowable integrator state
    double iGain,       // integral gain
        pGain,          // proportional gain
        dGain;          // derivative gain
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseCamera(const CameraConfiguration &configuration);

  virtual ~BaseCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void SetFeature(const Feature &feat, float value = 0);

  virtual float GetFeature(const Feature &feat) const;

  bool HasArtificialFramerate() const override;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  virtual float GetGainValue() const = 0;
  virtual void SetGainAuto() = 0;
  virtual void SetGainManual() = 0;
  virtual void SetGainValue(float value) = 0;

  virtual float GetGammaValue() const = 0;
  virtual void SetGammaValue(float value) = 0;

  virtual float GetExposureValue() const = 0;
  virtual void SetExposureValue(float value) = 0;

  virtual float GetSaturationValue() const = 0;
  virtual void SetSaturationValue(float value) = 0;

  virtual void SetShutterValue(float value) = 0;
  virtual void SetShutterAuto() = 0;
  virtual void SetShutterManual() = 0;
  virtual float GetShutterMode() const = 0;
  virtual float GetShutterValue() const = 0;

  virtual void SetFrameRateValue(float value) = 0;
  virtual float GetFrameRateValue() const = 0;

  virtual void SetWhiteBalanceAuto() = 0;
  virtual void SetWhiteBalanceManual() = 0;
  virtual float GetWhiteBalanceMode() const = 0;
  virtual void SetWhiteBalanceRedValue(float value) = 0;
  virtual void SetWhiteBalanceBlueValue(float value) = 0;
  virtual float GetWhiteBalanceRed() const = 0;
  virtual float GetWhiteBalanceBlue() const = 0;

  void Calibrate();

  float MSV(const cv::Mat &img, int nbrRegion);

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  CameraConfiguration config_;

  CameraUndistordMatrices undistord_matrix_;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  double UpdatePID(const std::shared_ptr<SPid> &pid, double error,
                   double position) ATLAS_NOEXCEPT;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool BaseCamera::HasArtificialFramerate() const { return false; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
