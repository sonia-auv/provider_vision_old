/**
 * \file	bilateral_filter.h
 * \author  Pierluc Bédard <pierlucbed@gmail.com>
 * \date	16/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_VISION_FILTERS_IN_RANGE_FILTER_H_
#define LIB_VISION_FILTERS_IN_RANGE_FILTER_H_

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
        _LUVlowL("LUVlowL", 0, 0, 255, parameters_),
        _LUVhighL("LUVhighL", 255, 0, 255, parameters_),
        _LUVlowU("LUVlowU", 0, 0, 255, parameters_),
        _LUVhighU("LUVhighU", 255, 0, 255, parameters_),
        _LUVlowV("LUVlowV", 0, 0, 255, parameters_),
        _LUVhighV("LUVhighV", 255, 0, 255, parameters_) {
    setName("InRange");
  }

  virtual ~InRange() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::Mat HSV, LUV;

      cv::cvtColor(image, HSV, cv::COLOR_RGB2HSV_FULL);
      cv::inRange(HSV, cv::Scalar(_HSVlowH.getValue(), _HSVlowS.getValue(),
                                  _HSVlowV.getValue()),
                  cv::Scalar(_HSVhighH.getValue(), _HSVhighS.getValue(),
                             _HSVhighV.getValue()),
                  HSV);

      cv::cvtColor(image, LUV, cv::COLOR_RGB2Luv);
      cv::inRange(LUV, cv::Scalar(_LUVlowL.getValue(), _LUVlowU.getValue(),
                                  _LUVlowV.getValue()),
                  cv::Scalar(_LUVhighL.getValue(), _LUVhighU.getValue(),
                             _LUVhighV.getValue()),
                  LUV);
      cv::bitwise_and(HSV, LUV, image);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _HSVlowH;
  IntegerParameter _HSVhighH;
  IntegerParameter _HSVlowS;
  IntegerParameter _HSVhighS;
  IntegerParameter _HSVlowV;
  IntegerParameter _HSVhighV;
  IntegerParameter _LUVlowL;
  IntegerParameter _LUVhighL;
  IntegerParameter _LUVlowU;
  IntegerParameter _LUVhighU;
  IntegerParameter _LUVlowV;
  IntegerParameter _LUVhighV;
};

}  // namespace vision_filter

#endif  // LIB_VISION_FILTERS_IN_RANGE_FILTER_H_
