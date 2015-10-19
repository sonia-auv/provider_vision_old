/**
 * \file	bilateral_filter.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_VISION_FILTERS_BILATERAL_FILTER_H_
#define LIB_VISION_FILTERS_BILATERAL_FILTER_H_

#include <lib_vision/filter.h>

namespace vision_filter {

class BilateralFilter : public Filter {
 public:
  //============================================================================
  // P U B L I C   C / D T O R

  explicit BilateralFilter(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _diameter("Diameter", -100, 0, 100, parameters_),
        _sigma_color("Sigm_color", 0, 0, 300, parameters_),
        _sigma_space("Sigma_space", 0, 0, 300, parameters_) {
    setName("BilateralFilter");
  }

  virtual ~BilateralFilter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::Mat blurred;
      cv::bilateralFilter(image, blurred, _diameter(), _sigma_color(),
                          _sigma_space());

      blurred.copyTo(image);
    }
  }

 private:
  // Params
  BooleanParameter _enable;

  IntegerParameter _diameter;

    IntegerParameter _sigma_color;

    IntegerParameter _sigma_space;
};

}  // namespace vision_filter

#endif  // LIB_VISION_FILTERS_BILATERAL_FILTER_H_
