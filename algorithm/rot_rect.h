/**
 * \file	rot_rect.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_ROTRECT_H_
#define VISION_FILTER_ROTRECT_H_

#define VISION_FILTER_TOP_LEFT 0
#define VISION_FILTER_TOP_RIGHT 4
#define VISION_FILTER_BOTTOM_LEFT 1
#define VISION_FILTER_BOTTOM_RIGHT 2

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>

// Rotated rect ensure that the height member is the longest one
// and the angle is in the direction of the height
// it also contains usefull method to play with rotated rectangle
class RotRect : public cv::RotatedRect {
 public:
  // Constructor/destructor
  RotRect(const std::vector<cv::Point> &edges);

  RotRect(const cv::RotatedRect &rotRect);

  RotRect(const RotRect &a);

  RotRect();

  ~RotRect();

  void drawRect(cv::Mat &out, cv::Scalar color, int thickness = 1);

  // Create the class with another rotated rect
  void swap(RotRect &a);

  RotRect &operator=(RotRect rotRect);

  RotRect &operator=(cv::RotatedRect rotRect);

  bool operator==(const RotRect &rotRect);

  cv::Point2f *getCorners();

 private:
  // Set height to the longest side of the rectangle and
  void setValues();

  cv::Point2f pts[4];
};

#endif
