/**
 * \file	type_and_const.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_TYPE_AND_CONST_
#define VISION_FILTER_TYPE_AND_CONST_

#include <opencv2/opencv.hpp>
// Definition of different type, different values used in our filters.

// Contour finding
typedef std::vector<cv::Point> contour_t;
typedef std::vector<contour_t> contourList_t;
typedef std::vector<cv::Vec4i> hierachy_t;
#define NEXT_CTR 0
#define PREV_CTR 1
#define FIRST_CHILD_CTR 2
#define PARENT_CTR 3

typedef std::vector<cv::Vec4i> defectuosity_t;

// Enum for the rotation function
enum rotationType { R_NONE = 0, R_90, R_180, R_270 };
enum symmetryType { S_NONE = 0, S_X_AXIS, S_Y_AXIS, S_BOTH };

#endif
