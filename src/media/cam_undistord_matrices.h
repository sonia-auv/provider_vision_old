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
  void InitMatrices(const std::string fullPath);

  void GetMatrices(cv::Mat &cameraMatrix, cv::Mat &distortionMatrix);

  inline bool IsCorrectionEnable() { return _matrices_founded; };

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat _camera_matrix, _distortion_matrix;

  std::string _xml_file_path;

  bool _matrices_founded;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void CameraUndistordMatrices::GetMatrices(cv::Mat &cameraMatrix,
                                                 cv::Mat &distortionMatrix) {
  _camera_matrix.copyTo(cameraMatrix);
  _distortion_matrix.copyTo(distortionMatrix);
}

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_UNDISTORD_MATRICES_H_
