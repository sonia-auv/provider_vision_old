/**
 * \file	cam_undistord_matrices.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/cam_undistord_matrices.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CameraUndistordMatrices::CameraUndistordMatrices() : matrices_founded_(false) {}

//------------------------------------------------------------------------------
//
CameraUndistordMatrices::~CameraUndistordMatrices() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CameraUndistordMatrices::InitMatrices(const std::string &fullPath) {
  matrices_founded_ = false;
  if (fullPath.empty()) return;

  cv::FileStorage fs(fullPath, cv::FileStorage::READ);

  if (fs.isOpened()) {
    fs["Camera_Matrix"] >> camera_matrix_;
    fs["Distortion_Coefficients"] >> distortion_matrix_;

    // Loading the matrices has initialized them. We know ( since it
    // is like this) that those matrices have those dimension. If they
    // are not right, then we did not successfully loaded them and we
    // should not use them.
    if (camera_matrix_.rows == 3 && camera_matrix_.cols == 3 &&
        distortion_matrix_.rows == 5 && distortion_matrix_.cols == 1) {
      matrices_founded_ = true;
      xml_file_path_ = fullPath;
    }
  }
}

}  // namespace provider_vision
