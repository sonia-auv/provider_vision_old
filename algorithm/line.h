/**
 * \file	line.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_LINE_H_
#define VISION_FILTER_LINE_H_

#include <opencv2/core/core.hpp>
#include <math.h>
#include <stdlib.h>

/*
 *
 * Basic definitnion of a line
 *
 */

class Line {
 public:
  // CSTR/DSTR
  Line(const cv::Point &start, const cv::Point &end);

  // Debug
  void draw(cv::Mat &img, cv::Scalar color);

  // Getters
  cv::Point getCenter();

  cv::Point getStart();

  cv::Point getEnd();

  float getAngle();

  float getLength();

 private:
  // start point is leftmosst point, end id farrigth point
  cv::Point center, start, end;
  // Degree
  float angle;
  float length;
};

bool lengthSort(Line a, Line b);

bool centerXSort(Line a, Line b);

bool centerYSort(Line a, Line b);

#endif /* LINE_H_ */
