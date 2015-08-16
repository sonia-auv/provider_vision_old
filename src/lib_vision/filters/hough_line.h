/**
 * \file  Sobel.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date  14/12/2014
 * \copyright Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_HOUGH_LINE_H_
#define VISION_FILTER_HOUGH_LINE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class HoughLine : public Filter {
 public:
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

}  // namespace vision_filter

#endif  // VISION_FILTER_SOBEL_H_
