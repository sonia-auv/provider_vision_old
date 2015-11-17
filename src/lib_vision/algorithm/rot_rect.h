/**
 * \file	rot_rect.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Pierluc Bédard <pierlucbed@gmail.com>
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
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
