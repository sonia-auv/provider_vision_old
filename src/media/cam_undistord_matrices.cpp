/**
 * \file	CAMConfig.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "media/cam_config.h"
#include "media/cam_undistord_matrices.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CameraUndistordMatrices::CameraUndistordMatrices() : _matrices_founded(false) {}

//------------------------------------------------------------------------------
//
CameraUndistordMatrices::~CameraUndistordMatrices() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CameraUndistordMatrices::InitMatrices(const std::string fullPath) {
  _matrices_founded = false;
  if (fullPath.empty()) return;

  cv::FileStorage fs(fullPath, cv::FileStorage::READ);

  if (fs.isOpened()) {
    fs["Camera_Matrix"] >> _camera_matrix;
    fs["Distortion_Coefficients"] >> _distortion_matrix;

    // Loading the matrices has initialized them. We know ( since it
    // is like this) that those matrices have those dimension. If they
    // are not right, then we did not successfully loaded them and we
    // should not use them.
    if (_camera_matrix.rows == 3 && _camera_matrix.cols == 3 &&
        _distortion_matrix.rows == 5 && _distortion_matrix.cols == 1) {
      _matrices_founded = true;
      _xml_file_path = fullPath;
    }
  }
}

}  // namespace vision_server
