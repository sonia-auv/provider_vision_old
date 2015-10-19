/**
 * \file	in_range.h
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

/**
 * The filter inRange check if array elements lie between certain value of HSV
 * and Luv. If it does, value of pixel = 1. If not, value of pixel = 0.
 */
class InRange : public Filter {
 public:
  //============================================================================
  // P U B L I C   C / D T O R

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

  virtual ~InRange() noexcept {}

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Override the execute function from the Filter class.
   * This is the function that is going to be called for processing the image.
   * This takes an image as a parameter and modify it with the filtered image.
   *
   * \param image The image to process.
   */
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
  //============================================================================
  // P R I V A T E   M E M B E R S

  /**
   * State if the filter is enabled or not.
   * This is being used by the vision server for calling the filter in the
   * filterchain.
   */
  BooleanParameter _enable;

  /**
   * Inclusive Hue lower boundary.
   */
  IntegerParameter _HSVlowH;

  /**
   * Inclusive Hue upper boundary.
   */
  IntegerParameter _HSVhighH;

  /**
   * Inclusive Saturation lower boundary.
   */
  IntegerParameter _HSVlowS;

  /**
   * Inclusive Saturation upper boundary.
   */
  IntegerParameter _HSVhighS;

  /**
   * Inclusive Value lower boundary.
   */
  IntegerParameter _HSVlowV;

  /**
   * Inclusive Value upper boundary.
   */
  IntegerParameter _HSVhighV;

  /**
   * Inclusive Lightness lower boundary.
   */
  IntegerParameter _LUVlowL;

  /**
   * Inclusive Lightness upper boundary.
   */
  IntegerParameter _LUVhighL;

  /**
   * Inclusive u lower boundary.
   */
  IntegerParameter _LUVlowU;

  /**
   * Inclusive u upper boundary.
   */
  IntegerParameter _LUVhighU;

  /**
   * Inclusive v lower boundary.
   */
  IntegerParameter _LUVlowV;

  /**
   * Inclusive v upper boundary.
   */
  IntegerParameter _LUVhighV;
};

}  // namespace vision_filter

#endif  // LIB_VISION_FILTERS_IN_RANGE_FILTER_H_
