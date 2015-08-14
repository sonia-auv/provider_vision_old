/**
 * \file	rot_rect.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/rot_rect.h>

RotRect::RotRect(const std::vector<cv::Point> &edges)
    : cv::RotatedRect(cv::minAreaRect(edges)) {
  setValues();
  this->points(this->pts);
}

//==============================================================
//
RotRect::RotRect() : cv::RotatedRect() {}

//==============================================================
//
RotRect::RotRect(const cv::RotatedRect &rotRect) : cv::RotatedRect(rotRect) {
  setValues();
  this->points(this->pts);
}

//==============================================================
//
RotRect::RotRect(const RotRect &a) : cv::RotatedRect(a) {
  for (int i = 0; i < 4; i++) this->pts[i] = a.pts[i];
}

//==============================================================
//
RotRect::~RotRect() {}

//==============================================================
//
void RotRect::drawRect(cv::Mat &out, cv::Scalar color, int thickness) {
  cv::line(out, pts[0], pts[1], color, thickness);
  cv::line(out, pts[1], pts[2], color, thickness);
  cv::line(out, pts[2], pts[3], color, thickness);
  cv::line(out, pts[3], pts[0], color, thickness);
}

//==============================================================
//
// Set height to the longest side of the rectangle and
void RotRect::setValues() {
  float in_angle = this->angle;
  float out_angle = in_angle;
  // angle is consider in of height in opencv
  // since height always is not always on the longest size,
  // make sure always return the longest size in the height
  // and makes the angle follow.
  if (this->size.width > this->size.height) {
    std::swap(this->size.width, this->size.height);
    if (in_angle < 0)
      out_angle = 90 + in_angle;
    else
      out_angle = -(90 - in_angle);
  }
  this->angle = out_angle;
}

//==============================================================
//
void RotRect::swap(RotRect &a) {
  std::swap(a.angle, angle);
  std::swap(a.center, center);
  std::swap(a.pts, pts);
  std::swap(a.size, size);
}

//==============================================================
//
RotRect &RotRect::operator=(RotRect rotRect) {
  swap(rotRect);
  return *this;
}

//==============================================================
//
RotRect &RotRect::operator=(cv::RotatedRect rotRect) {
  RotRect slRotRect(rotRect);
  swap(slRotRect);
  return *this;
}

//==============================================================
//
bool RotRect::operator==(const RotRect &rotRect) {
  bool result = true;
  if (rotRect.center != this->center) {
    result = false;
  }
  if (rotRect.size != this->size) {
    result = false;
  }
  if (abs(rotRect.angle - rotRect.angle) > 0.5) {
    result = false;
  }
  return result;
}

//==============================================================
//
cv::Point2f *RotRect::getCorners() { return this->pts; }
