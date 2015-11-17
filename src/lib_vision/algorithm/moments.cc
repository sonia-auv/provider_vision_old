/**
 * \file	moments.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/moments.h>

//==============================================================
//
Moments::Moments(cv::Mat image, bool binary) {
  if (image.channels() != 1) {
    cv::Mat image2;
    cv::cvtColor(image, image2, CV_BGR2GRAY);
    image2.copyTo(image);
  }

  // Gets the moment by opencv moment function
  this->cvMoments = cv::moments(image, binary);

  this->massCenter = cv::Point2f(this->cvMoments.m10 / this->cvMoments.m00,
                                 this->cvMoments.m01 / this->cvMoments.m00);

  // Here, remember that the mome are calculated on an image. If the image
  // was extract from a rotatedRect, it means the coordinate are in the angle
  // of the rotatedRect. X and Y axis of the image are rotated of angle degree
  this->realCenter = cv::Point(image.cols / 2, image.rows / 2);

  this->xDistanceFromCenter = massCenter.x - realCenter.x;
  this->yDistanceFromCenter = massCenter.y - realCenter.y;
}

//==============================================================
//
Moments::Moments() {
  this->cvMoments = cv::Moments();
  this->massCenter = cv::Point(-1, -1);
  this->realCenter = cv::Point(-1, -1);
  this->xDistanceFromCenter = 0.0f;
  this->yDistanceFromCenter = 0.0f;
}

//==============================================================
//
Moments::~Moments() {}

//==============================================================
//
void Moments::operator=(Moments moments) {
  this->cvMoments = moments.cvMoments;
  this->massCenter = moments.massCenter;
  this->realCenter = moments.realCenter;
  this->xDistanceFromCenter = moments.xDistanceFromCenter;
  this->yDistanceFromCenter = moments.yDistanceFromCenter;
}
