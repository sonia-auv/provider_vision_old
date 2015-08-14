/**
 * \file	moments.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_MOMENTS_H_
#define VISION_FILTER_MOMENTS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>

class Moments {
 public:
  Moments(cv::Mat image, bool binary);

  Moments();

  ~Moments();

  void operator=(Moments moments);

  // Points are in local coordinate, meaning that if the image was taken
  // from a rotated rectangle, the x and y a rotated too!
  cv::Point realCenter;
  cv::Point massCenter;
  float yDistanceFromCenter;
  float xDistanceFromCenter;

  cv::Moments cvMoments;
};

#endif
