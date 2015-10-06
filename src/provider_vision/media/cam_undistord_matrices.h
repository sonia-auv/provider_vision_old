/**
 * \file	CameraID.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_CAM_UNDISTORD_MATRICES_H_
#define PROVIDER_VISION_CAM_UNDISTORD_MATRICES_H_

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "lib_atlas/macros.h"

namespace vision_server {

/**
 * CameraID is a little class to handle the link between GUID and name.
 * We can then identify a camera by its name but the system uses its GUID.
 */
class CameraUndistordMatrices {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CameraUndistordMatrices>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  CameraUndistordMatrices();

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
inline bool CameraUndistordMatrices::IsCorrectionEnable() {
  return matrices_founded_;
}

//------------------------------------------------------------------------------
//
inline void CameraUndistordMatrices::CorrectInmage(const cv::Mat &in,
                                                   cv::Mat &out) const {
  if (matrices_founded_) {
    cv::undistort(in, out, camera_matrix_, distortion_matrix_);
  } else {
    in.copyTo(out);
  }
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_CAM_UNDISTORD_MATRICES_H_
