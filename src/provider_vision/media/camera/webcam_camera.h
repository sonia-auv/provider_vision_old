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

#ifndef PROVIDER_VISION_MEDIA_CAMERA_WEBCAM_CAMERA_H_
#define PROVIDER_VISION_MEDIA_CAMERA_WEBCAM_CAMERA_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_camera.h"

namespace provider_vision {

/**
 * WebcamCamera is the object for handling webcams.
 * For now it consider that the default camera is a webcam.
 * There is no other check than that. It is useful for debugging the server.
 */
class WebcamCamera : public BaseMedia, private cv::VideoCapture {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<WebcamCamera>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  WebcamCamera();

  /**
   * By giving the id of the camera, OpenCv will open it from it's own,
   * do not try to start the webcam if you already started it with
   * this constructor.
   */
  explicit WebcamCamera(int webcamIdx);

  virtual ~WebcamCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Method override from Media
   */
  void Open() override;

  /**
   * Method override from Media
   */
  void Close() override;

  /**
   * Method override from Media
   */
  void SetStreamingModeOn() override;

  /**
   * Method override from Media
   */
  void SetStreamingModeOff() override;

  /**
   * Method override from Media
   */
  void NextImage(cv::Mat &image) override;
};

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_WEBCAM_CAMERA_H_
