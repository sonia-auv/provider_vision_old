/**
 * \file	contour.h
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

#ifndef CONTOUR_H_
#define CONTOUR_H_

#include <opencv2/opencv.hpp>

class Contour {
 public:

  typedef std::vector<cv::Point> ContourVec;

  Contour(const std::vector<cv::Point> &ctr);
  Contour(const cv::RotatedRect &rect);

  // Approximate contours and merges vertex together
  // using cv::approxContour
  void Approximate(double accuracy);
  void ApproximateBySize();

  // Draw contour in the image.
  void DrawContours(cv::Mat &image, cv::Scalar color, int thickness);

  // Vector overload
  size_t size();
  std::vector<cv::Point> Get();
  cv::Point operator[](unsigned int index);
  std::vector<cv::Point> _contour;
};

//-----------------------------------------------------------------------------
//
inline void Contour::Approximate(double accuracy) {
  std::vector<cv::Point> output;
  cv::approxPolyDP(_contour, output, accuracy, false);
  std::swap(_contour, output);
}

//-----------------------------------------------------------------------------
//
inline void Contour::ApproximateBySize() {
  double arc_length = 0.1 * cv::arcLength(_contour, true);
  std::vector<cv::Point> output;
  cv::approxPolyDP(_contour, output, arc_length, false);
  std::swap(_contour, output);
}

//-----------------------------------------------------------------------------
//
inline void Contour::DrawContours(cv::Mat &image, cv::Scalar color,
                                  int thickness) {
  std::vector<ContourVec> ctrs;
  ctrs.push_back(_contour);
  cv::drawContours(image, ctrs, -1, color, thickness);
}

//-----------------------------------------------------------------------------
//
inline size_t Contour::size() { return _contour.size(); }

//-----------------------------------------------------------------------------
//
inline std::vector<cv::Point> Contour::Get()
{
  return _contour;
}

//-----------------------------------------------------------------------------
//
inline cv::Point Contour::operator[](unsigned int index) {
  return _contour[index];
}

#endif /* LIB_VISION_SRC_LIB_VISION_ALGORITHM_CONTOUR_FINDER_H_ */
