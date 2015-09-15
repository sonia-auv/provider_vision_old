/**
 * \file	CameraID.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_UNDISTORD_MATRICES_H_
#define VISION_SERVER_CAM_UNDISTORD_MATRICES_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <stdint.h>
#include <string>
#include <opencv2/opencv.hpp>

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * CameraID is a little class to handle the link between GUID and name.
 * We can then identify a camera by its name but the system uses its GUID.
 */
class CameraUndistordMatrices {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit CameraUndistordMatrices();

  virtual ~CameraUndistordMatrices();

  //==========================================================================
  // P U B L I C   M E T H O D S
  void InitMatrices(const std::string &fullPath);

  void GetMatrices(cv::Mat &cameraMatrix, cv::Mat &distortionMatrix);

  bool IsCorrectionEnable();

  void CorrectInmage(const cv::Mat &in, cv::Mat &out) const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat camera_matrix_, distortion_matrix_;

  std::string xml_file_path_;

  bool matrices_founded_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void CameraUndistordMatrices::GetMatrices(cv::Mat &cameraMatrix,
                                                 cv::Mat &distortionMatrix) {
  camera_matrix_.copyTo(cameraMatrix);
  distortion_matrix_.copyTo(distortionMatrix);
}

//------------------------------------------------------------------------------
//
inline bool
CameraUndistordMatrices::IsCorrectionEnable() { return matrices_founded_; };

//------------------------------------------------------------------------------
//
inline void
CameraUndistordMatrices::CorrectInmage(const cv::Mat &in, cv::Mat &out) const {
  if (matrices_founded_) {
    cv::undistort(in, out, camera_matrix_, distortion_matrix_);
  } else {
    in.copyTo(out);
  }
}

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_UNDISTORD_MATRICES_H_
