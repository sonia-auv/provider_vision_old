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

#include <lib_atlas/macros.h>
#ifndef OS_DARWIN

#include <ros/ros.h>
#include <string>
#include "provider_vision/media/camera/gige_camera.h"

namespace provider_vision {

const std::string GigeCamera::CAM_TAG = "[GigE Camera]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
GigeCamera::GigeCamera(const CameraConfiguration &config)
    : BaseCamera(config), gige_camera_(nullptr) {}

//------------------------------------------------------------------------------
//
GigeCamera::~GigeCamera() {
  GevAbortImageTransfer(&gige_camera_);
  GevFreeImageTransfer(&gige_camera_);
  GevCloseCamera(&gige_camera_);
  GevApiInitialize();
  _CloseSocketAPI();
}

//------------------------------------------------------------------------------
//
void GigeCamera::Open() {
  UINT16 status;

  if (IsOpened()) {
    throw std::logic_error("The media is already started");
  }

  std::lock_guard<std::mutex> guard(cam_access_);

  try {
    std::string str = CameraConfiguration::name_;
    char *name = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), name);
    name[str.size()] = '\0';  // don't forget the terminating 0
    for (int i = 0; i < 5; i++) {
      status = GevOpenCameraByName(name, GevControlMode, &gige_camera_);
      if (status == 0) {
        i = 5;
        ROS_INFO_STREAM_NAMED("[GigE Driver]", "%s opened successfully" << str);
      } else {
        ROS_INFO_NAMED("[GigE Driver]",
                       "Unable to open camera. Retrying. Try: %d/5", i + 1);
        sleep(10);
      }
    }
    // don't forget to free the string after finished using it
    delete[] name;
    if (status != 0) {
      throw std::runtime_error(GevGetFormatString(status));
    }
    status = GevInitGenICamXMLFeatures(gige_camera_, TRUE);

    if (status != 0) {
      throw std::runtime_error(GevGetFormatString(status));
    }
  } catch (std::exception &e) {
    ROS_ERROR("%s", e.what());
    ROS_ERROR("Error opening GigE camera");
  }
  UINT32 format = fMtBayerRG8;
  UINT32 width = 0;
  UINT32 height = 0;
  UINT32 x_offset = 0;
  UINT32 y_offset = 0;

  try {
    SetCameraParams();
    status = GevSetImageParameters(
        gige_camera_, (UINT32)width_, (UINT32)height_,
        (UINT32)x_offset_, (UINT32)y_offset_,
        (UINT32)format_);
    status = GevGetImageParameters(gige_camera_, &width, &height, &x_offset,
                                   &y_offset, &format);

    UINT32 maxDepth = GetPixelSizeInBytes(format);

    // Allocate image buffers

    PUINT8 bufAddress[DMA_BUFFER];

    UINT32 size = maxDepth * width * height;
    for (int i = 0; i < DMA_BUFFER; i++) {
      bufAddress[i] = (PUINT8)malloc(size);
      memset(bufAddress[i], 0, size);
    }
    status = GevInitImageTransfer(gige_camera_, Asynchronous, DMA_BUFFER,
                                  bufAddress);
  }
  CATCH_GENAPI_ERROR(status);
  status_ = Status::OPEN;
}

//------------------------------------------------------------------------------
//
void GigeCamera::Close() {
  if (!IsOpened()) {
    throw std::logic_error("The media is not started");
  }

  std::lock_guard<std::mutex> guard(cam_access_);

  bool close_result = true;
  if (status_ == Status::STREAMING) {
    StopStreaming();
  }

  GEV_STATUS status = GevCloseCamera(&gige_camera_);

  if (status != GEVLIB_OK) {
    close_result = false;
  }

  close_result ? status_ = Status::CLOSE : status_ = Status::ERROR;
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetStreamingModeOn() {
  GEV_STATUS status;
  std::lock_guard<std::mutex> guard(cam_access_);

  status = GevStartImageTransfer(gige_camera_, -1);

  if (status == GEVLIB_ERROR_INVALID_HANDLE) {
    status_ = Status::ERROR;
    throw std::runtime_error("Invalid handle. Cannot set streaming mode on.");
  } else if (status == GEV_STATUS_BUSY) {
    status_ = Status::ERROR;
    throw std::runtime_error("Camera is busy. Cannot set streaming mode on.");
  }
  atlas::MilliTimer::Sleep(2500);
  if (!white_balance_manual_) SetWhiteBalanceMode(1);

  status_ = Status::STREAMING;
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetStreamingModeOff() {
  std::lock_guard<std::mutex> guard(cam_access_);

  GEV_STATUS status = GevStopImageTransfer(gige_camera_);
  if (status != GEVLIB_OK) {
    status_ = Status::ERROR;
    throw std::runtime_error(
        "Invalid handle. The camera could not be stopped.");
  }

  status_ = Status::OPEN;
  // Here stopping timer just in case... Should already be closed....
  std::lock_guard<std::mutex> guard2(timer_access_);
}

//------------------------------------------------------------------------------
//
void GigeCamera::NextImage(cv::Mat &img) {
  GEV_BUFFER_OBJECT *frame = NULL;
  timer_access_.lock();
  acquisition_timer_.Sleep(3);
  acquisition_timer_.Start();
  timer_access_.unlock();

  cam_access_.lock();
  GEV_STATUS status = GevWaitForNextImage(gige_camera_, &frame, 1000);
  cam_access_.unlock();
  timer_access_.lock();
  atlas::MilliTimer::Sleep(3);
  timer_access_.unlock();

  if (status != 0) {
    status_ = Status::ERROR;
    throw std::runtime_error("Cannot get next image.");
  }

  if (frame != NULL) {
    try {
      cv::Mat tmp = cv::Mat(frame->h, frame->w, CV_8UC1, frame->address);
      //      undistord_matrix_.CorrectInmage(tmp, img);
      tmp.copyTo(img);
      cv::cvtColor(tmp, img, CV_BayerRG2RGB);
      BalanceWhite(img);
    } catch (cv::Exception &e) {
      status_ = Status::ERROR;
      throw;
    }
  }

  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    throw std::runtime_error(
        "The image is empty, there is a problem with the media");
  }
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetCameraParams() {
  SetAutoBrightnessMode(auto_brightness_);
  atlas::MilliTimer::Sleep(100);
  if (auto_brightness_) {
    SetAutoBrightnessTarget(auto_brightness_target_);
    atlas::MilliTimer::Sleep(100);
    SetAutoBrightnessTargetVariation(auto_brightness_target_variation_);
    atlas::MilliTimer::Sleep(100);
  }
  if (gain_manual_) {
    if (auto_brightness_) {
      SetGainMode(0);
    }
    SetGainValue((float)gain_);
  } else if (auto_brightness_)
    SetGainMode(true);
  atlas::MilliTimer::Sleep(100);
  if (exposure_manual_) {
    if (auto_brightness_) {
      SetExposureMode(0);
    }
    atlas::MilliTimer::Sleep(100);
    SetExposureValue((float)exposure_);
  } else if (auto_brightness_)
    SetExposureMode(true);
  atlas::MilliTimer::Sleep(100);

  GevSetImageParameters(gige_camera_, (UINT32)width_,
                        (UINT32)height_, (UINT32)x_offset_,
                        (UINT32)y_offset_, (UINT32)format_);
  atlas::MilliTimer::Sleep(100);
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetAutoBrightnessMode(int value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("autoBrightnessMode");
  ptrEnumNode->SetIntValue(value);
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetAutoBrightnessTarget(int value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CIntegerPtr ptrIntNode = Camera->_GetNode("autoBrightnessTarget");
  ptrIntNode->SetValue(value);
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetAutoBrightnessTargetVariation(int value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CIntegerPtr ptrIntNode =
      Camera->_GetNode("autoBrightnessTargetRangeVariation");
  ptrIntNode->SetValue(value);
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetGainMode(bool mode) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("GainAuto");
  if (mode == FeatureMode::AUTO) {
    ptrEnumNode->SetIntValue(2);
  } else if (mode == FeatureMode::MANUAL) {
    ptrEnumNode->SetIntValue(0);
  }
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetGainMode() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("GainAuto");
  return static_cast<bool>(ptrEnumNode->GetIntValue());
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetGainValue() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrGain = Camera->_GetNode("Gain");
  return (float)ptrGain->GetValue();
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetGainValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrGain = Camera->_GetNode("Gain");
  ptrGain->SetValue(value);
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetExposureMode(bool mode) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrExposureAuto = Camera->_GetNode("ExposureAuto");
  GenApi::CEnumerationPtr ptrExposureMode = Camera->_GetNode("ExposureMode");

  if (mode == FeatureMode::AUTO) {
    ptrExposureAuto->SetIntValue(2);
    atlas::MilliTimer::Sleep(100);
    ptrExposureMode->SetIntValue(2);
  } else if (mode == FeatureMode::MANUAL) {
    ptrExposureAuto->SetIntValue(0);
    atlas::MilliTimer::Sleep(100);
    ptrExposureMode->SetIntValue(0);
  }
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetExposureMode() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrExposureMode = Camera->_GetNode("ExposureMode");
  auto mode = ptrExposureMode->GetIntValue();

  if(mode == 2) {
    return FeatureMode::AUTO;
  } else {
    return  FeatureMode::MANUAL;
  }
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetExposureValue() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrExposureTime = Camera->_GetNode("ExposureTime");
  return (float)ptrExposureTime->GetValue();
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetExposureValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrExposureTime = Camera->_GetNode("ExposureTime");
  ptrExposureTime->SetValue(value);
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetFrameRateValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrFrameRate = Camera->_GetNode("AcquisitionFrameRate");
  ptrFrameRate->SetValue(value, true);
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetFrameRateValue() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrFrameRate = Camera->_GetNode("AcquisitionFrameRate");
  return (float)ptrFrameRate->GetValue();
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetWhiteBalanceMode(bool mode) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("BalanceWhiteAuto");

  if (mode == FeatureMode::AUTO) {
    ptrEnumNode->SetIntValue(1);
    atlas::MilliTimer::Sleep(100);
    GenApi::CCommandPtr ptrWhiteBalanceCmd =
        Camera->_GetNode("balanceWhiteAutoOnDemandCmd");
    ptrWhiteBalanceCmd->Execute();
  } else if (mode == FeatureMode::MANUAL) {
    ptrEnumNode->SetIntValue(0);
  }
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetWhiteBalanceMode() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("BalanceWhiteAuto");
  return static_cast<bool>(ptrEnumNode->GetIntValue());
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetWhiteBalanceRatio() const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
  return (double)ptrWhiteBalanceRatio->GetValue();
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetWhiteBalanceRatio(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
  ptrWhiteBalanceRatio->SetValue(value);
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetWhiteBalanceRed() const {
  ROS_WARN("The feature WhiteBalance is not available on GigE cameras.");
  return -1;
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetWhiteBalanceRedValue(double value) {
  ROS_WARN("The feature WhiteBalance is not available on GigE cameras.");
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetWhiteBalanceBlueValue(double value) {
  ROS_WARN("The feature WhiteBalance is not available on GigE cameras.");
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetWhiteBalanceBlue() const {
  ROS_WARN("The feature WhiteBalance is not available on GigE cameras.");
  return -1;
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetGammaValue() const {
  ROS_WARN("The feature GammaValue is not available on GigE cameras.");
  return -1;
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetGammaValue(double value) {
  ROS_WARN("The feature GammaValue is not available on GigE cameras.");
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetSaturationValue() const {
  ROS_WARN("The feature SaturationValue is not available on GigE cameras.");
  return -1;
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetSaturationValue(double value) {
  ROS_WARN("The feature SaturationValue is not available on GigE cameras.");
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetShutterValue(double value) {
  ROS_WARN("The feature ShutterValue is not available on GigE cameras.");
}

//------------------------------------------------------------------------------
//
void GigeCamera::SetShutterMode(bool) {
  ROS_WARN("The feature ShutterMode is not available on GigE cameras.");
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetShutterMode() const {
  ROS_WARN("The feature ShutterMode is not available on GigE cameras.");
  return 0;
}

//------------------------------------------------------------------------------
//
double GigeCamera::GetShutterValue() const {
  ROS_WARN("The feature ShutterValue is not available on GigE cameras.");
  return -1;
}

//------------------------------------------------------------------------------
//
void GigeCamera::BalanceWhite(cv::Mat mat) {
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3 * 256 * sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar *ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols * mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total) vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total) vmax[i] -= 1;
    if (vmax[i] < 255 - 1) vmax[i] += 1;
  }

  for (int y = 0; y < mat.rows; ++y) {
    uchar *ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j]) val = vmin[j];
        if (val > vmax[j]) val = vmax[j];
        ptr[x * 3 + j] =
            static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }
}

}  // namespace provider_vision

#endif  // OS_DARWIN
