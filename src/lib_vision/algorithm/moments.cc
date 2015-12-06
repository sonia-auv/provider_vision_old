/**
 * \file	moments.cc
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

#include <lib_vision/algorithm/moments.h>

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Moments::Moments(cv::Mat image, bool binary) {
  if (image.channels() != 1) {
    cv::Mat image2;
    cv::cvtColor(image, image2, CV_BGR2GRAY);
    image2.copyTo(image);
  }

  // Gets the moment by opencv moment function
  cvMoments = cv::moments(image, binary);

  massCenter =
      cv::Point2f(cvMoments.m10 / cvMoments.m00, cvMoments.m01 / cvMoments.m00);

  // Here, remember that the mome are calculated on an image. If the image
  // was extract from a rotatedRect, it means the coordinate are in the angle
  // of the rotatedRect. X and Y axis of the image are rotated of angle degree
  realCenter = cv::Point(image.cols / 2, image.rows / 2);

  xDistanceFromCenter = massCenter.x - realCenter.x;
  yDistanceFromCenter = massCenter.y - realCenter.y;
}

//------------------------------------------------------------------------------
//
Moments::Moments() {
  cvMoments = cv::Moments();
  massCenter = cv::Point(-1, -1);
  realCenter = cv::Point(-1, -1);
  xDistanceFromCenter = 0.0f;
  yDistanceFromCenter = 0.0f;
}

//------------------------------------------------------------------------------
//
Moments::~Moments() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Moments::operator=(Moments moments) {
  cvMoments = moments.cvMoments;
  massCenter = moments.massCenter;
  realCenter = moments.realCenter;
  xDistanceFromCenter = moments.xDistanceFromCenter;
  yDistanceFromCenter = moments.yDistanceFromCenter;
}
