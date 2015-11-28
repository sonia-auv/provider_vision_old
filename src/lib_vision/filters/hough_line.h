/**
 * \file	hough_line.h
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

#ifndef LIB_VISION_FILTERS_HOUGH_LINE_H_
#define LIB_VISION_FILTERS_HOUGH_LINE_H_

#include <lib_vision/filter.h>

namespace lib_vision {

class HoughLine : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<HoughLine>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit HoughLine(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _rho("Rho", 1.0f, 0.0f, 1000.0f, parameters_),
        _theta("Theta", 1.0f, 0.0f, 1000.0f, parameters_),
        _min_length("Min_length", 1, 0, 1000, parameters_),
        _max_gap("Max_gap", 1, 0, 1000, parameters_),
        _threshold("Threshold", 1, 0, 1000, parameters_) {
    setName("HoughLine");
  }

  virtual ~HoughLine() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }

      std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(image, lines, _rho(), _theta(), _threshold(),
                      _min_length(), _max_gap());

      cv::Mat drawing_image(image.rows, image.cols, CV_8UC3,
                            cv::Scalar::all(0));
      for (const auto &line : lines) {
        cv::line(drawing_image, cv::Point(line[0], line[1]),
                 cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 3);
      }
      cv::cvtColor(drawing_image, image, CV_BGR2GRAY);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  DoubleParameter _rho, _theta, _min_length, _max_gap;
  IntegerParameter _threshold;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_SOBEL_H_
