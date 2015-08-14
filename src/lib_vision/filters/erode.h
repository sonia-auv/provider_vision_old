/**
 * \file	Erode.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_ERODE_H_
#define VISION_FILTER_ERODE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class Erode : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Erode(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _use_square_kernel("Square_kernel", true, parameters_),
        _kernel_type("Kernel_type", 0, 0, 2, parameters_),
        _kernel_size_x("Width", 1, 0, 20, parameters_),
        _kernel_size_y("Height", 1, 0, 20, parameters_),
        _iteration("Iteration", 1, 0, 20, parameters_),
        _anchor(-1, -1) {
    setName("Erode");
  }

  virtual ~Erode() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      int kernel_type = 0;
      switch (_kernel_type()) {
        case 0:
          kernel_type = cv::MORPH_RECT;
          break;
        case 1:
          kernel_type = cv::MORPH_ELLIPSE;
          break;
        case 2:
          kernel_type = cv::MORPH_CROSS;
          break;
      }

      cv::Size size(_kernel_size_x() * 2 + 1,
                    (_use_square_kernel() ? _kernel_size_x() * 2 + 1
                                          : _kernel_size_y() * 2 + 1));
      cv::Mat kernel = cv::getStructuringElement(kernel_type, size, _anchor);

      cv::erode(image, image, kernel, _anchor, _iteration());
    }
  }

 private:
  // Params
  BooleanParameter _enable, _use_square_kernel;
  IntegerParameter _kernel_type;
  IntegerParameter _kernel_size_x, _kernel_size_y;
  IntegerParameter _iteration;

  const cv::Point _anchor;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_ERODE_H_
