/**
 * \file	line.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/line.h>

//=============================================================================
//
Line::Line(const cv::Point &start, const cv::Point &end)
    : start(start), end(end), angle(0) {
  if (this->start.x > this->end.x) std::swap(this->start, this->end);

  int yDiff = abs(start.y - end.y);
  int xDiff = abs(start.x - end.x);

  if (start.y > end.y)
    this->center.y = end.y + yDiff / 2;
  else
    this->center.y = start.y + yDiff / 2;

  if (start.x > end.x)
    this->center.x = end.x + xDiff / 2;
  else
    this->center.x = start.x + xDiff / 2;

  // inversion in the start end for x y is because y is positive
  // downward.
  float at = atan2(static_cast<double>(start.y - end.y),
                   static_cast<double>(end.x - start.x));
  this->angle = at / (2 * M_PI) * 360;

  this->length = sqrt(pow((start.y - end.y), 2) + pow((start.y - end.y), 2));
}

//=============================================================================
//
void Line::draw(cv::Mat &img, cv::Scalar color) {
  cv::line(img, this->start, this->end, color, 4, 8);
}

//=============================================================================
//
cv::Point Line::getCenter() { return this->center; }

//=============================================================================
//
cv::Point Line::getStart() { return this->start; }

//=============================================================================
//
cv::Point Line::getEnd() { return this->end; }

//=============================================================================
//
float Line::getAngle() { return this->angle; }

//=============================================================================
//
float Line::getLength() { return this->length; }

//=============================================================================
//
bool lengthSort(Line a, Line b) { return a.getLength() > b.getLength(); }

//=============================================================================
//
bool centerXSort(Line a, Line b) { return a.getCenter().x > b.getCenter().x; }

//=============================================================================
//
bool centerYSort(Line a, Line b) { return a.getCenter().y > b.getCenter().y; }
