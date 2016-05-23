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
