/**
 * \file	Threshold.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_THRESHOLD_H_
#define VISION_FILTER_THRESHOLD_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class Threshold : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Threshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _type("Threshold_type", 1, 0, 5, parameters_,
              "0=BIN, 1=BIN_INV, 2=TRUNC, 3=TOZERO, 4=TOZERO_INV 5=OTSU"),
        _max("Max_value", 100, 0, 255, parameters_) {
    setName("Threshold");
  }

  virtual ~Threshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      if (image.depth() != CV_8U) {
        image.convertTo(image, CV_8U);
      }

      int threshold_type = CV_THRESH_BINARY;
      switch (_type()) {
        case 0:
          threshold_type = CV_THRESH_BINARY;
          break;
        case 1:
          threshold_type = CV_THRESH_BINARY_INV;
          break;
        case 2:
          threshold_type = CV_THRESH_TRUNC;
          break;
        case 3:
          threshold_type = CV_THRESH_TOZERO;
          break;
        case 4:
          threshold_type = CV_THRESH_TOZERO_INV;
          break;
        case 5:
          threshold_type = CV_THRESH_OTSU;
          break;
        default:
          threshold_type = CV_THRESH_BINARY;
          break;
      }
      cv::threshold(image, image, _max(), 255, threshold_type);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _type, _max;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_THRESHOLD_H_
