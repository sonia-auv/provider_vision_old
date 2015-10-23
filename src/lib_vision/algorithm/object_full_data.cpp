/**
 * \file	object_full_data.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_full_data.h>

ObjectFullData::ObjectFullData(const cv::Mat &originalImage,
                               const cv::Mat &binaryImage,
                               const Contour &contour)
    : ObjectBasicData(originalImage, binaryImage, contour) {}
