/**
 * \file	webcam_camera.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_CAMERA_WEBCAM_CAMERA_H_
#define PROVIDER_VISION_MEDIA_CAMERA_WEBCAM_CAMERA_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "provider_vision/media/camera/base_camera.h"
#include "provider_vision/utils/config.h"

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

  WebcamCamera() noexcept;

  /**
   * By giving the id of the camera, OpenCv will open it from it's own,
   * do not try to start the webcam if you already started it with
   * this constructor.
   */
  explicit WebcamCamera(int webcamIdx) noexcept;

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
