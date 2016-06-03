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

#include "provider_vision/media/camera/webcam_camera.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera() noexcept
    : BaseMedia("Webcam"), cv::VideoCapture() {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
WebcamCamera::WebcamCamera(int webcamIdx) noexcept
    : BaseMedia("Webcam"), cv::VideoCapture(webcamIdx) {
  if (isOpened()) {
    status_ = Status::OPEN;
  }
}

//------------------------------------------------------------------------------
//
WebcamCamera::~WebcamCamera() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool WebcamCamera::Open() {
  // Already been open at constructor.
  if (!IsOpened()) {
    open(0);
    status_ = Status::OPEN;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::Close() {
  if (IsOpened()) {
    release();
    status_ = Status::CLOSE;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::SetStreamingModeOn() {
  status_ = Status::STREAMING;
  return true;
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::SetStreamingModeOff() {
  status_ = Status::OPEN;
  return true;
}

//------------------------------------------------------------------------------
//
bool WebcamCamera::NextImage(cv::Mat &image) {
  if (isOpened() && (status_ == Status::OPEN || status_ == Status::STREAMING)) {
    operator>>(image);
    return true;
  } else {
    image = cv::Mat().clone();
    return false;
  }
}

}  // namespace provider_vision
