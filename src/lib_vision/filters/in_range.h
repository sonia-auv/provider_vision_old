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
        _HSVlowH("HSVLowH", 0, 0, 255, parameters_),
        _HSVhighH("HSVHighH", 255, 0, 255, parameters_),
        _HSVlowS("HSVLowS", 0, 0, 255, parameters_),
        _HSVhighS("HSVHighS", 255, 0, 255, parameters_),
        _HSVlowV("HSVLowV", 0, 0, 255, parameters_),
        _HSVhighV("HSVHighV", 255, 0, 255, parameters_),
        _LUVlowL(),
        _LUVhighL(),
        _LUVlow{
    setName("InRange");
  }

  virtual ~InRange() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::cvtColor(image, image, cv::COLOR_RGB2HSV_FULL);
      cv::inRange(image, cv::Scalar(_HSVlowH(), _HSVlowS(), _HSVlowV()),
                  cv::Scalar(_HSVhighH(), _HSVhighS(), _HSVhighV()), image);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _HSVlowH, _HSVhighH, _HSVlowS, _HSVhighS, _HSVlowV, _HSVhighV;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_INRANGE_H_
