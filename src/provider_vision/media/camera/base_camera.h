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

#ifndef PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
#define PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_

#include <boost/any.hpp>
#include <memory>
#include <mutex>
#include "provider_vision/config.h"
#include "provider_vision/media/cam_undistord_matrices.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/camera_calibrator.h"

namespace provider_vision {

/**
 * Every camera (i.e. not video or image) should inherit this instead of
 * Media class.
 * This enables feature and open/close stuff.
 */
class BaseCamera : public BaseMedia, protected CameraConfiguration {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseCamera>;

  /**
   * When calling with node, so string, please use
   * the enum name, ex. "FRAMERATE" for FEATURE::FRAMERATE
   */
  enum class Feature {
    SHUTTER_AUTO = 0,
    SHUTTER_VALUE,
    GAIN_AUTO,
    GAIN_VALUE,
    WHITE_BALANCE_AUTO,
    WHITE_BALANCE_RED_VALUE,
    WHITE_BALANCE_BLUE_VALUE,
    // FRAMERATE_MODE,
    FRAMERATE_VALUE,
    // GAMMA_MODE,
    GAMMA_VALUE,
    EXPOSURE_AUTO,
    EXPOSURE_VALUE,
    // SATURATION_MODE,
    SATURATION_VALUE,
    INVALID_FEATURE,
    AUTOBRIGHTNESS_AUTO,
    AUTOBRIGHTNESS_TARGET,
    AUTOBRIGHTNESS_VARIATION,
    WHITE_BALANCE_EXECUTE,
    WHITE_BALANCE_GREEN_VALUE
  };

  struct FeatureMode {
    static constexpr bool AUTO = true;
    static constexpr bool MANUAL = false;
  };

  // When getter cannot acheive their task, they will return
  // this value.
  static const double INVALID_DOUBLE;
  static const float INVALID_FLOAT;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseCamera(const CameraConfiguration &configuration);

  virtual ~BaseCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  bool SetFeature(const Feature &feat, const boost::any &value);

  bool GetFeature(const Feature &feat, boost::any &value) const;

  bool HasArtificialFramerate() const override;

  void PublishCameraFeatures() const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  virtual bool SetGainMode(bool value) = 0;
  virtual bool SetGainValue(double value) = 0;
  virtual bool GetGainMode(bool &value) const = 0;
  virtual bool GetGainValue(double &value) const = 0;

  virtual bool GetGammaValue(double &value) const = 0;
  virtual bool SetGammaValue(double value) = 0;

  virtual bool SetExposureMode(bool value) = 0;
  virtual bool SetExposureValue(double value) = 0;
  virtual bool GetExposureMode(bool &value) const = 0;
  virtual bool GetExposureValue(double &value) const = 0;

  virtual bool GetSaturationValue(double &value) const = 0;
  virtual bool SetSaturationValue(double value) = 0;

  virtual bool SetShutterValue(double value) = 0;
  virtual bool SetShutterMode(bool value) = 0;
  virtual bool GetShutterMode(bool &value) const = 0;
  virtual bool GetShutterValue(double &value) const = 0;

  virtual bool SetFrameRateValue(double value) = 0;
  virtual bool GetFrameRateValue(double &value) const = 0;

  virtual bool SetWhiteBalanceMode(bool value) = 0;
  virtual bool GetWhiteBalanceMode(bool &value) const = 0;
  virtual bool SetWhiteBalanceRedValue(double value) = 0;
  virtual bool SetWhiteBalanceBlueValue(double value) = 0;
  virtual bool SetWhiteBalanceGreenValue(double value) = 0;
  virtual bool GetWhiteBalanceRed(double &value) const = 0;
  virtual bool GetWhiteBalanceBlue(double &value) const = 0;
  virtual bool GetWhiteBalanceGreen(double &value) const = 0;

  virtual bool GetWhiteBalanceRatio(double &value) const = 0;
  virtual bool SetWhiteBalanceRatio(double value) = 0;
  virtual bool SetAutoBrightnessMode(double value) = 0;
  virtual bool SetAutoBrightnessTarget(double value) = 0;
  virtual bool SetAutoBrightnessTargetVariation(double value) = 0;
  virtual bool GetAutoBrightnessTargetVariation(int &value) const = 0;
  virtual bool GetAutoBrightnessMode(bool &value) const = 0;
  virtual bool GetAutoBrightnessTarget(int &value) const = 0;

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  ros::Publisher feature_pub_;

  // CameraCalibrator calibrator_;

  CameraUndistordMatrices undistord_matrix_;

  /// We want the CameraCalibrator to access the features of the camera directly
  /// without having to call the GetFeature method.
  friend class CameraCalibrator;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool BaseCamera::HasArtificialFramerate() const { return false; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
