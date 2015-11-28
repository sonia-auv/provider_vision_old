/**
 * \file	line.h
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

#ifndef LIB_VISION_ALGORITHM_LINE_H_
#define LIB_VISION_ALGORITHM_LINE_H_

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
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Line>;

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

#endif // LIB_VISION_ALGORITHM_LINE_H_
