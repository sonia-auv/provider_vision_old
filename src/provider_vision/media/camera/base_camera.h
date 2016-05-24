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

#include <memory>
#include <mutex>
#include <boost/any.hpp>
#include "provider_vision/media/camera_calibrator.h"
#include "provider_vision/config.h"
#include "provider_vision/media/cam_undistord_matrices.h"
#include "provider_vision/media/camera/base_media.h"

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
    SHUTTER_MODE,
    SHUTTER_VALUE,
    GAIN_MODE,
    GAIN_VALUE,
    WHITE_BALANCE_MODE,
    WHITE_BALANCE_RED_VALUE,
    WHITE_BALANCE_BLUE_VALUE,
    // FRAMERATE_MODE,
    FRAMERATE_VALUE,
    // GAMMA_MODE,
    GAMMA_VALUE,
    EXPOSURE_MODE,
    EXPOSURE_VALUE,
    // SATURATION_MODE,
    SATURATION_VALUE
  };

  struct FeatureMode {
    static constexpr bool AUTO = true;
    static constexpr bool MANUAL = false;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseCamera(const CameraConfiguration &configuration);

  virtual ~BaseCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void SetFeature(const Feature &feat, const boost::any &value);

  void GetFeature(const Feature &feat, boost::any &value) const;

  bool HasArtificialFramerate() const override;

  void PublishCameraFeatures() const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  virtual void SetGainMode(bool) = 0;
  virtual void SetGainValue(double value) = 0;
  virtual bool GetGainMode() const = 0;
  virtual double GetGainValue() const = 0;

  virtual double GetGammaValue() const = 0;
  virtual void SetGammaValue(double value) = 0;

  virtual void SetExposureMode(bool) = 0;
  virtual bool GetExposureMode() const = 0;
  virtual double GetExposureValue() const = 0;
  virtual void SetExposureValue(double value) = 0;

  virtual double GetSaturationValue() const = 0;
  virtual void SetSaturationValue(double value) = 0;

  virtual void SetShutterValue(double value) = 0;
  virtual void SetShutterMode(bool) = 0;
  virtual bool GetShutterMode() const = 0;
  virtual double GetShutterValue() const = 0;

  virtual void SetFrameRateValue(double value) = 0;
  virtual double GetFrameRateValue() const = 0;

  virtual void SetWhiteBalanceMode(bool) = 0;
  virtual bool GetWhiteBalanceMode() const = 0;
  virtual void SetWhiteBalanceRedValue(double value) = 0;
  virtual void SetWhiteBalanceBlueValue(double value) = 0;
  virtual double GetWhiteBalanceRed() const = 0;
  virtual double GetWhiteBalanceBlue() const = 0;

  float GetLimunanceMSV() const noexcept;
  float GetSaturationMSV() const noexcept;

  void Calibrate(cv::Mat const &img);

  float CalculateMSV(const cv::Mat &img, int nbrRegion);

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  ros::Publisher feature_pub_;

  CameraUndistordMatrices undistord_matrix_;

 private:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

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
  struct CalibrationFeatureValues {
    double gain;
    double gamma;
    double exposure;
    double saturation;
  };

  //==========================================================================
  // P R I V A T E   M E T H O D S

  double UpdatePID(SPid &pid, double error, double position) ATLAS_NOEXCEPT;

  cv::Mat CalculateLuminanceHistogram(const cv::Mat &img) const;
  cv::Mat CalculateSaturationHistogram(const cv::Mat &img) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  CalibrationFeatureValues current_features_;

  CameraCalibrator calibrator_;

  double gain_lim_;
  double exposure_lim_;

  /// We are going to lock every access to the features of the camera as we are
  /// certainly going to be accessed through a different thread (ros service)
  mutable std::mutex features_mutex_;

  /// The last values of the MSV calculated during the calibration of the
  /// cameras.
  float msv_lum_;
  float msv_sat_;

  SPid gamma_pid_, gain_pid_, exposure_pid_, saturation_pid_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool BaseCamera::HasArtificialFramerate() const { return false; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_BASE_CAMERA_H_
