/**
 * \file	WebcamCamera.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_WEBCAM_H_
#define PROVIDER_VISION_CAM_WEBCAM_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_camera.h"

namespace vision_server {

/**
 * WebcamCamera is the object for handling webcams.
 * For now it consider that the default camera is a webcam.
 * There is no other check than that. It is useful for debugging the server.
 */
class WebcamCamera : public BaseCamera, private cv::VideoCapture {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<WebcamCamera>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  WebcamCamera() noexcept;

  explicit WebcamCamera(int webcamIdx) noexcept;

  virtual ~WebcamCamera();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Method override from Media
   */
  void Start() override;

  /**
   * Method override from Media
   */
  void Stop() override;

  /**
   * Method override from Media
   */
  void NextImage(cv::Mat &image) override;

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
  void SetFeature(const Feature &feat, float value) override;

  /**
   * Method override from Media
   */
  float GetFeature(const Feature &feat) const override;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_WEBCAM_H_
