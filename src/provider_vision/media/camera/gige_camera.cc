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

const char *GigeCamera::CAM_TAG = "[GigE Camera]";

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
bool GigeCamera::Open() {
  UINT16 status = 0;

  if (IsOpened()) {
    ROS_INFO_NAMED(CAM_TAG, "The media is already started");
    return true;
  }

  cam_access_.lock();

  try {
    std::string str = CameraConfiguration::name_;
    char *name = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), name);
    name[str.size()] = '\0';  // don't forget the terminating 0
    for (int i = 0; i < 5; i++) {
      status = GevOpenCameraByName(name, GevControlMode, &gige_camera_);
      if (status == 0) {
        i = 5;
        ROS_INFO_NAMED(CAM_TAG, "opened successfully %s", str.c_str());
      } else {
        ROS_INFO_NAMED(CAM_TAG, "Unable to open camera. Retrying. Try: %d/5",
                       i + 1);
        sleep(10);
      }
    }
    // don't forget to free the string after finished using it
    delete[] name;
    if (status != 0) {
      ROS_ERROR_NAMED(CAM_TAG, "Error while opening the camera %s",
                      GevGetFormatString(status));
      cam_access_.unlock();
      return false;
    }
    status = GevInitGenICamXMLFeatures(gige_camera_, TRUE);

    if (status != 0) {
      ROS_ERROR_NAMED(CAM_TAG, "Error while getting the camera feature %s",
                      GevGetFormatString(status));
      cam_access_.unlock();
      return false;
    }
  } catch (std::exception &e) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while opening the camera. GIGE: %s EXECPTION: %s",
                    GevGetFormatString(status), e.what());
    cam_access_.unlock();
    return false;
  }
  cam_access_.unlock();
  UINT32 format = 0X01080009;
  UINT32 width = 0;
  UINT32 height = 0;
  UINT32 x_offset = 0;
  UINT32 y_offset = 0;

  try {
    status = GevSetImageParameters(gige_camera_, (UINT32)width_,
                                   (UINT32)height_, (UINT32)x_offset_,
                                   (UINT32)y_offset_, (UINT32)format_);
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
  } catch (std::exception &e) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while opening the camera. GIGE: %s EXECPTION: %s",
                    GevGetFormatString(status), e.what());
    return false;
  }

  try
  {
    //enable turbo mode
    if(IsTurboAvailable())
    {
      UINT32 val = 1;
      GevSetFeatureValue(gige_camera_, "transferTurboMode", sizeof(UINT32), &val);
      ROS_INFO_NAMED(CAM_TAG, "turbo enabled for %s", CameraConfiguration::name_.c_str());
    }
    else
    {
      ROS_INFO_NAMED(CAM_TAG, "turbo not available for %s", CameraConfiguration::name_.c_str());
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_NAMED(CAM_TAG,
                    "Error while enabling turbo. GIGE: %s EXECPTION: %s",
                    GevGetFormatString(status), e.what());
    return false;
  }
  
  status_ = Status::OPEN;
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::Close() {
  if (!IsOpened()) {
    ROS_INFO_NAMED(CAM_TAG, "The media is not started");
    return true;
  }

  std::lock_guard<std::mutex> guard(cam_access_);

  bool close_result = true;
  if (status_ == Status::STREAMING) {
    close_result = StopStreaming();
  }

  GEV_STATUS status = GevCloseCamera(&gige_camera_);

  if (status != GEVLIB_OK) {
    close_result = false;
  }

  close_result ? status_ = Status::CLOSE : status_ = Status::ERROR;
  return close_result;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetStreamingModeOn() {
  GEV_STATUS status;
  std::lock_guard<std::mutex> guard(cam_access_);

  status = GevStartImageTransfer(gige_camera_, -1);

  if (status == GEVLIB_ERROR_INVALID_HANDLE) {
    status_ = Status::ERROR;
    ROS_ERROR_NAMED(CAM_TAG, "Invalid handle. Cannot set streaming mode on.");
    return false;
  } else if (status == GEV_STATUS_BUSY) {
    status_ = Status::ERROR;
    ROS_ERROR_NAMED(CAM_TAG, "Camera is busy. Cannot set streaming mode on.");
    return false;
  }

  status_ = Status::STREAMING;
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetStreamingModeOff() {
  std::lock_guard<std::mutex> guard(cam_access_);
  int status;
  try {
    status = GevStopImageTransfer(gige_camera_);
  } CATCH_GENAPI_ERROR(status)
  if (status) {
    ROS_ERROR("Exception on closing image transfer");
  }

  if (status != GEVLIB_OK) {
    status_ = Status::ERROR;
    ROS_ERROR_NAMED(CAM_TAG,
                    "Invalid handle. The camera could not be stopped.");
    return false;
  }

  status_ = Status::OPEN;
  // Here stopping timer just in case... Should already be closed....
  std::lock_guard<std::mutex> guard2(timer_access_);
  try {
    acquisition_timer_.Pause();
    acquisition_timer_.Reset();
  } catch (std::exception &e) {
    ROS_ERROR_NAMED(CAM_TAG, "Exception on timer handling %s", e.what());
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::NextImage(cv::Mat &img) {
  GEV_BUFFER_OBJECT *frame = NULL;
  timer_access_.lock();
  acquisition_timer_.Sleep(3);
  acquisition_timer_.Start();
  timer_access_.unlock();

  cam_access_.lock();
  GEV_STATUS status;
  try {
    status = GevWaitForNextImage(gige_camera_, &frame, 1000);
  } CATCH_GENAPI_ERROR(status) {
  }

  cam_access_.unlock();
  timer_access_.lock();
  atlas::MilliTimer::Sleep(3);
  timer_access_.unlock();

  if (status != GEV_STATUS_SUCCESS || frame == nullptr) {
    status_ = Status::ERROR;
    ROS_ERROR_NAMED(CAM_TAG, "Cannot get next image. Status is: %d", status);
    throw std::exception();
    return false;
  }

  if (frame != NULL) {
    try {
      cv::Mat tmp = cv::Mat(frame->h, frame->w, CV_8UC1, frame->address);
      tmp.copyTo(img);
      cv::cvtColor(tmp, img, CV_BayerRG2RGB);
    } catch (cv::Exception &e) {
      status_ = Status::ERROR;
      ROS_ERROR_NAMED(CAM_TAG, "Error on opencv image transformation %s",
                      e.what());
      return false;
    }
  }

  if (img.empty() || img.size().height == 0 || img.size().height == 0) {
    ROS_ERROR_NAMED(CAM_TAG,
                    "The image is empty, there is a problem with the media");
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetCameraParams() {
  SetAutoBrightnessMode(auto_brightness_auto_);
  atlas::MilliTimer::Sleep(100);
  if (auto_brightness_auto_) {
    SetAutoBrightnessTarget(auto_brightness_target_);
    atlas::MilliTimer::Sleep(100);
    SetAutoBrightnessTargetVariation(auto_brightness_target_variation_);
    atlas::MilliTimer::Sleep(100);
  }
  if (!gain_auto_) {
    if (auto_brightness_auto_) {
      SetGainMode(0);
    }
    SetGainValue((float)gain_);
  } else if (auto_brightness_auto_)
    SetGainMode(true);
  atlas::MilliTimer::Sleep(100);
  if (!exposure_auto_) {
    if (auto_brightness_auto_) {
      SetExposureMode(0);
    }
    atlas::MilliTimer::Sleep(100);
    SetExposureValue((float)exposure_);
  } else if (auto_brightness_auto_)
    SetExposureMode(true);
  atlas::MilliTimer::Sleep(100);

  GevSetImageParameters(gige_camera_, (UINT32)width_, (UINT32)height_,
                        (UINT32)x_offset_, (UINT32)y_offset_, (UINT32)format_);
  atlas::MilliTimer::Sleep(100);

  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetAutoBrightnessMode(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("autoBrightnessMode");
  if (ptrEnumNode) {
    GevStopImageTransfer(gige_camera_);
    atlas::MilliTimer::Sleep(100);
    ptrEnumNode->SetIntValue(static_cast<int>(value));
    ROS_INFO("%i", (int)value);
    atlas::MilliTimer::Sleep(100);
    if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Auto brightness is not available on this"
                   " cameras.");
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetAutoBrightnessMode(bool &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("autoBrightnessMode");
  if (ptrEnumNode) {
    value = (bool)(ptrEnumNode->GetCurrentEntry()->GetNumericValue());
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Auto brightness is not available on this"
                   " cameras.");
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetAutoBrightnessTarget(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));

  GenApi::CIntegerPtr ptrIntNode = Camera->_GetNode("autoBrightnessTarget");
  if (ptrIntNode) {
    bool ab;
    GetAutoBrightnessMode(ab);
    if (ab) ptrIntNode->SetValue(value);
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Auto brightness is not available on this"
                   " cameras.");
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetAutoBrightnessTarget(int &value) const {
  bool ab;
  GetAutoBrightnessMode(ab);
  value = -1;
  if (ab) {
    GenApi::CNodeMapRef *Camera =
        static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));

    GenApi::CIntegerPtr ptrIntNode = Camera->_GetNode("autoBrightnessTarget");
    if (ptrIntNode) {
      value = (int)ptrIntNode->GetValue();

    } else {
      ROS_WARN_NAMED(CAM_TAG,
                     "The feature Auto brightness is not available on this"
                     " cameras.");
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetAutoBrightnessTargetVariation(double value) {
  bool ab;
  GetAutoBrightnessMode(ab);
  if (ab) {
    GenApi::CNodeMapRef *Camera =
        static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
    GenApi::CIntegerPtr ptrIntNode =
        Camera->_GetNode("autoBrightnessTargetRangeVariation");
    if (ptrIntNode) {
      bool ab;
      GetAutoBrightnessMode(ab);
      if (ab) ptrIntNode->SetValue(value);
    } else {
      ROS_WARN_NAMED(CAM_TAG,
                     "The feature Auto brightness is not available on this"
                     " cameras.");
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetAutoBrightnessTargetVariation(int &value) const {
  bool ab;
  GetAutoBrightnessMode(ab);
  value = -1;
  if (ab) {
    GenApi::CNodeMapRef *Camera =
        static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
    GenApi::CIntegerPtr ptrIntNode =
        Camera->_GetNode("autoBrightnessTargetRangeVariation");
    if (ptrIntNode) {
      value = (int)ptrIntNode->GetValue();
    } else {
      ROS_WARN_NAMED(CAM_TAG,
                     "The feature Auto brightness is not available on this"
                     " cameras.");
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetGainMode(bool mode) {
  bool ABmode;
  GetAutoBrightnessMode(ABmode);
  if (!ABmode) return true;

  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("GainAuto");

  if (ptrEnumNode) {
    GevStopImageTransfer(gige_camera_);
    atlas::MilliTimer::Sleep(100);
    if (mode) {
      ptrEnumNode->SetIntValue(2);
      atlas::MilliTimer::Sleep(100);
    } else {  // Manual
      ptrEnumNode->SetIntValue(0);
      atlas::MilliTimer::Sleep(100);
    }
    if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
    atlas::MilliTimer::Sleep(100);
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Gain mode is not available on this"
                   " cameras.");
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetGainMode(bool &value) const {
  bool ab;
  GetAutoBrightnessMode(ab);
  if (ab) {
    GenApi::CNodeMapRef *Camera =
        static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
    GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("GainAuto");
    if (ptrEnumNode) {
      // value = static_cast<bool>(ptrEnumNode->GetIntValue());
      value = (bool)ptrEnumNode->GetIntValue();
    } else {
      ROS_WARN_NAMED(CAM_TAG,
                     "The feature Gain mode is not available on this"
                     " cameras.");
    }
  } else
    value = false;

  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetGainValue(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrGain = Camera->_GetNode("Gain");
  if (ptrGain) {
    value = (float)ptrGain->GetValue();
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Gain value is not available on this"
                   " cameras.");
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetGainValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrGain = Camera->_GetNode("Gain");
  if (ptrGain) {
    ptrGain->SetValue(value);
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Gain value is not available on this"
                   " cameras.");
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetExposureMode(bool mode) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrExposureAuto = Camera->_GetNode("ExposureAuto");
  GenApi::CEnumerationPtr ptrExposureMode = Camera->_GetNode("ExposureMode");
  bool ABmode;
  GetAutoBrightnessMode(ABmode);
  if (!ABmode) return true;
  if (ptrExposureAuto && ptrExposureMode) {
    GevStopImageTransfer(gige_camera_);
    atlas::MilliTimer::Sleep(100);
    if (mode == FeatureMode::AUTO) {
      ptrExposureAuto->SetIntValue(2);
      atlas::MilliTimer::Sleep(100);
    } else {  // Manual mode
      ptrExposureAuto->SetIntValue(0);
      atlas::MilliTimer::Sleep(100);
      ptrExposureMode->SetIntValue(0);
    }
    if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
    atlas::MilliTimer::Sleep(100);
  } else {
    ROS_WARN_NAMED(CAM_TAG,
                   "The feature Exposure Mode is not available on this"
                   " cameras.");
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetExposureMode(bool &value) const {
  bool ab;
  GetAutoBrightnessMode(ab);
  if (ab) {
    GenApi::CNodeMapRef *Camera =
        static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
    GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("ExposureAuto");
    value = (bool)ptrEnumNode->GetIntValue();

  } else
    value = false;
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetExposureValue(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrExposureTime = Camera->_GetNode("ExposureTime");
  value = (double)ptrExposureTime->GetValue();
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetExposureValue(double value) {
  bool mode;
  GetExposureMode(mode);
  if (!mode) {
    GenApi::CNodeMapRef *Camera =
        static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
    GenApi::CFloatPtr ptrExposureTime = Camera->_GetNode("ExposureTime");
    ptrExposureTime->SetValue(value);
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetFrameRateValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrFrameRate = Camera->_GetNode("AcquisitionFrameRate");
  ptrFrameRate->SetValue(value, true);
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetFrameRateValue(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrFrameRate = Camera->_GetNode("AcquisitionFrameRate");
  value = (double)ptrFrameRate->GetValue();
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetWhiteBalanceMode(bool mode) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("BalanceWhiteAuto");
  if (mode) {
    GevStopImageTransfer(gige_camera_);
    atlas::MilliTimer::Sleep(100);
    ptrEnumNode->SetIntValue(1);
    atlas::MilliTimer::Sleep(100);
    GenApi::CCommandPtr ptrWhiteBalanceCmd =
        Camera->_GetNode("balanceWhiteAutoOnDemandCmd");
    ptrWhiteBalanceCmd->Execute();
    atlas::MilliTimer::Sleep(100);
    if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
  } else {
    GevStopImageTransfer(gige_camera_);
    atlas::MilliTimer::Sleep(100);
    ptrEnumNode->SetIntValue(0);
    atlas::MilliTimer::Sleep(100);
    if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
    GetWhiteBalanceBlue(white_balance_blue_);
    GetWhiteBalanceRed(white_balance_red_);
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetWhiteBalanceMode(bool &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("BalanceWhiteAuto");
  value = static_cast<bool>(ptrEnumNode->GetIntValue());
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetWhiteBalanceRatio(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
  value = (double)ptrWhiteBalanceRatio->GetValue();
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetWhiteBalanceRatio(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
  ptrWhiteBalanceRatio->SetValue(value);
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetWhiteBalanceRed(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode =
      Camera->_GetNode("BalanceRatioSelector");
  if (ptrEnumNode) {
    ptrEnumNode->SetIntValue(0);
    GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
    if (ptrWhiteBalanceRatio) {
      value = ptrWhiteBalanceRatio->GetValue();
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetWhiteBalanceRedValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GevStopImageTransfer(gige_camera_);
  atlas::MilliTimer::Sleep(100);
  GenApi::CEnumerationPtr ptrEnumNode =
      Camera->_GetNode("BalanceRatioSelector");
  if (ptrEnumNode) {
    ptrEnumNode->SetIntValue(0);
    GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
    if (ptrWhiteBalanceRatio) {
      ptrWhiteBalanceRatio->SetValue(value);
    }
  }
  if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);

  return true;
}

//------------------------------------------------------------------------------
//

bool GigeCamera::SetWhiteBalanceGreenValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GevStopImageTransfer(gige_camera_);
  atlas::MilliTimer::Sleep(100);
  GenApi::CEnumerationPtr ptrEnumNode =
      Camera->_GetNode("BalanceRatioSelector");
  if (ptrEnumNode) {
    ptrEnumNode->SetIntValue(1);
    GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
    if (ptrWhiteBalanceRatio) {
      ptrWhiteBalanceRatio->SetValue(value);
    }
  }
  if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetWhiteBalanceGreen(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode =
      Camera->_GetNode("BalanceRatioSelector");
  if (ptrEnumNode) {
    ptrEnumNode->SetIntValue(1);
    GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
    if (ptrWhiteBalanceRatio) {
      value = ptrWhiteBalanceRatio->GetValue();
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetWhiteBalanceBlueValue(double value) {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GevStopImageTransfer(gige_camera_);
  atlas::MilliTimer::Sleep(100);
  GenApi::CEnumerationPtr ptrEnumNode =
      Camera->_GetNode("BalanceRatioSelector");
  if (ptrEnumNode) {
    ptrEnumNode->SetIntValue(2);
    GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
    if (ptrWhiteBalanceRatio) {
      ptrWhiteBalanceRatio->SetValue(value);
    }
  }
  if (status_ == Status::STREAMING) GevStartImageTransfer(gige_camera_, -1);
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetWhiteBalanceBlue(double &value) const {
  GenApi::CNodeMapRef *Camera =
      static_cast<GenApi::CNodeMapRef *>(GevGetFeatureNodeMap(gige_camera_));
  GenApi::CEnumerationPtr ptrEnumNode =
      Camera->_GetNode("BalanceRatioSelector");
  if (ptrEnumNode) {
    ptrEnumNode->SetIntValue(2);
    GenApi::CFloatPtr ptrWhiteBalanceRatio = Camera->_GetNode("BalanceRatio");
    if (ptrWhiteBalanceRatio) {
      value = ptrWhiteBalanceRatio->GetValue();
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetGammaValue(double &value) const {
  (void)value;
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature GammaValue is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetGammaValue(double value) {
  (void)value;
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature GammaValue is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetSaturationValue(double &value) const {
  (void)value;
  ROS_WARN_NAMED(
      CAM_TAG, "The feature SaturationValue is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetSaturationValue(double value) {
  (void)value;
  ROS_WARN_NAMED(
      CAM_TAG, "The feature SaturationValue is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetShutterValue(double value) {
  (void)value;
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature ShutterValue is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::SetShutterMode(bool value) {
  (void)value;
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature ShutterMode is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetShutterMode(bool &value) const {
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature ShutterMode is not available on GigE cameras.");
  return false;
}

//------------------------------------------------------------------------------
//
bool GigeCamera::GetShutterValue(double &value) const {
  (void)value;
  ROS_WARN_NAMED(CAM_TAG,
                 "The feature ShutterValue is not available on GigE cameras.");
  return false;
}

bool GigeCamera::IsTurboAvailable()
{
  int type;
	UINT32 val = 0;
	
	if ( 0 == GevGetFeatureValue( gige_camera_, "transferTurboCurrentlyAbailable",  &type, sizeof(UINT32), &val) )
	{
		return (val != 0);
	}
	else
	{
		char pxlfmt_str[64] = {0};

		GevGetFeatureValueAsString( gige_camera_, "PixelFormat", &type, sizeof(pxlfmt_str), pxlfmt_str);
		if ( 0 != GevSetFeatureValueAsString( gige_camera_, "transferTurboCapabilitySelector", pxlfmt_str) )
		{
			return false; 
		}
		else
		{
			return true;
		}
	}
	return false;
}

}  // namespace provider_vision

#endif  // OS_DARWIN
