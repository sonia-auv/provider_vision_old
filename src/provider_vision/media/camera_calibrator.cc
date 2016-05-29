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

#include <provider_vision/media/camera_calibrator.h>
#include "provider_vision/media/camera/base_camera.h"
#include <boost/lexical_cast.hpp>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CameraCalibrator::CameraCalibrator(const ros::NodeHandle &nh,
                                   const std::string &name)
    : ConfigurationParser(nh, "/provider_vision/camera_pid/"),
      name_(name),
      gamma_pid_(),
      gain_pid_(),
      exposure_pid_(),
      saturation_pid_(),
      msv_lum_(),
      msv_sat_(),
      msv_uniform_(2.5),
      gain_lim_(),
      exposure_lim_() {
  DeserializeConfiguration(name_);
}

//------------------------------------------------------------------------------
//
CameraCalibrator::~CameraCalibrator() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CameraCalibrator::Calibrate(BaseCamera *camera, cv::Mat img) {
  std::lock_guard<std::mutex> guard(features_mutex_);
  auto l_hist = CalculateLuminanceHistogram(img);

  msv_lum_ = CalculateMSV(l_hist, 5);
  double shutter = camera->GetShutterValue();
  try {
    if (msv_lum_ > 2.6){
      shutter -= 100;
    }
    else if(msv_lum_ < 2.4){
      shutter += 100;
    }
    camera->SetShutterValue(shutter);

  } catch (const std::exception &e) {
    ROS_ERROR("Error in calibrate camera: %s.\n", e.what());
  }
}

//------------------------------------------------------------------------------
//
cv::Mat CameraCalibrator::CalculateLuminanceHistogram(
    const cv::Mat &img) const {
  static const int hist_size{256};
  static const float range[] = {0, 255};
  static const float *histRange{range};

  // Get the LUV image from the RGB image in input parameter.
  //cv::Mat luvImg;
  //cvtColor(img, luvImg, CV_RGB2Luv);

  // Splitting the LUV Image into 3 channels in the luv_planes.
  std::vector<cv::Mat> luv_planes;
  cv::split(img, luv_planes);

  // Calculate the histogram of luminance by sending the first element of
  // the plane (the L channel)
  cv::Mat l_hist;
  cv::calcHist(&luv_planes[0], 1, 0, cv::Mat(), l_hist, 1, &hist_size,
               &histRange, true, false);
  return l_hist;
}

//------------------------------------------------------------------------------
//
cv::Mat CameraCalibrator::CalculateSaturationHistogram(
    const cv::Mat &img) const {
  static const int hist_size{256};
  static const float range[] = {0, 255};
  static const float *histRange{range};

  // Get the LUV image from the RGB image in input parameter.
  cv::Mat hsv_img;
  cvtColor(img, hsv_img, CV_RGB2HSV_FULL);

  // Splitting the LUV Image into 3 channels in the luv_planes.
  std::vector<cv::Mat> hsv_planes;
  cv::split(hsv_img, hsv_planes);

  // Calculate the histogram of saturation by sending the first element of
  // the plane (the L channel)
  cv::Mat hist;
  cv::calcHist(&hsv_planes[1], 1, 0, cv::Mat(), hist, 1, &hist_size, &histRange,
               true, false);
  return hist;
}

//------------------------------------------------------------------------------
//
float CameraCalibrator::CalculateMSV(const cv::Mat &img, int nbrRegion) {
  float num = 0.f, deno = 0.f;
  int inter = std::ceil(256 / nbrRegion);
  for (int j = 0; j < nbrRegion; ++j) {
    float num_buff = 0.f;
    for (int i = j * inter; i < (j + 1) * inter; ++i) {
      deno += img.at<float>(i);
      num_buff += img.at<float>(i);
    }
    num += num_buff * (j + 1);
  }
  return num / deno;
}

//------------------------------------------------------------------------------
//
double CameraCalibrator::GetLimunanceMSV() const noexcept {
  std::lock_guard<std::mutex> guard(features_mutex_);
  return msv_lum_;
}

//------------------------------------------------------------------------------
//
double CameraCalibrator::GetSaturationMSV() const noexcept {
  std::lock_guard<std::mutex> guard(features_mutex_);
  return msv_sat_;
}


//------------------------------------------------------------------------------
//
void CameraCalibrator::DeserializeConfiguration(const std::string &name) {
  double a;
  FindParameter(name_ + "/gamma_i_gain", a);
  FindParameter(name_ + "/gamma_p_gain", a);
  FindParameter(name_ + "/gamma_d_gain", a);

  FindParameter(name_ + "/gain_i_gain", a);
  FindParameter(name_ + "/gain_p_gain", a);
  FindParameter(name_ + "/gain_d_gain", a);

  FindParameter(name_ + "/exposure_i_gain", a);
  FindParameter(name_ + "/exposure_p_gain", a);
  FindParameter(name_ + "/exposure_d_gain", a);

  FindParameter(name_ + "/saturation_i_gain", a);
  FindParameter(name_ + "/saturation_p_gain", a);
  FindParameter(name_ + "/saturation_d_gain", a);

  FindParameter(name_ + "/gain_lim", gain_lim_);
  FindParameter(name_ + "/exposure_lim", exposure_lim_);
}

}  // namespace provider_vision
