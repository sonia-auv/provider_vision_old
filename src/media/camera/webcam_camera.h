/**
 * \file	CAMWebcam.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_WEBCAM_H_
#define VISION_SERVER_CAM_WEBCAM_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <opencv2/opencv.hpp>
#include "config.h"
#include "media/camera/base_camera.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * CAMWebcam is the object for handling webcams.
 * For now it consider that the default camera is a webcam.
 * There is no other check than that. It is useful for debugging the server.
 */
class CAMWebcam : public BaseCamera, private cv::VideoCapture {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  CAMWebcam();

  CAMWebcam(int webcamIdx);

  virtual ~CAMWebcam();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /** Method override from Media */
  bool Start() override;

  /** Method override from Media */
  bool Stop() override;

  /** Method override from Media */
  bool NextImage(cv::Mat &image) override;

  /** Method override from Media */
  bool Open() override;

  /** Method override from Media */
  bool Close() override;

  /** Method override from Media */
  bool SetFeature(const Feature &feat, float value) override;

  /** Method override from Media */
  float GetFeature(const Feature &feat) const override;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_WEBCAM_H_
