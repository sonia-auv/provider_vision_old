/*
 * contour_finder.h
 *
 *  Created on: Aug 24, 2015
 *      Author: jeremie
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
  void DrawContours(cv::Mat &image, const cv::Scalar &color, int thickness);

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
inline void Contour::DrawContours(cv::Mat &image, const cv::Scalar &color,
                                  int thickness) {
  std::vector<Contour> ctrs;
  ctrs.push_back(_contour);
  cv::polylines(image, ctrs, true, color, thickness);
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
