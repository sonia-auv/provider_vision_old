/**
 * \file	InRange.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_INRANGE_H_
#define VISION_FILTER_INRANGE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class InRange : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit InRange(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _lowH("LowH", 0, 0, 179, parameters_),
        _highH("HighH", 178, 0, 179, parameters_),
        _lowS("LowS", 0, 0, 255, parameters_),
        _highS("HighS", 254, 0, 255, parameters_),
        _lowV("LowV", 0, 0, 255, parameters_),
        _highV("HighV", 254, 0, 255, parameters_) {
    setName("InRange");
  }

  virtual ~InRange() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::cvtColor(image, image, cv::COLOR_RGB2HSV);
      cv::inRange(image, cv::Scalar(_lowH(), _lowS(), _lowV()),
                  cv::Scalar(_highH(), _highS(), _highV()), image);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _lowH, _highH, _lowS, _highS, _lowV, _highV;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_INRANGE_H_
