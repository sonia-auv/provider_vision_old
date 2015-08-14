/**
 * \file  InRange.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date  14/12/2014
 * \copyright Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_ADAPTIVE_THRESHOLD_H_
#define VISION_FILTER_ADAPTIVE_THRESHOLD_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class AdaptiveThreshold : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit AdaptiveThreshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _method("Method", 0, 0, 1, parameters_, "0=Gaussian 1=Mean"),
        _threshold_type("Threshold_type", 0, 0, 1, parameters_,
                        "0=BIN, 1=BIN_INV"),
        _block_size("Size", 1,1,40, parameters_),
         _c_param("C_param", 0.0f, -255.0f, 255.0f, parameters_){
    setName("AdaptiveThreshold");
  }

  virtual ~AdaptiveThreshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if ( _enable() ) {
      if( image.channels() > 1)
      {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      int size = _block_size()*2+1;
      int method = _method() == 0 ? cv::ADAPTIVE_THRESH_GAUSSIAN_C : cv::ADAPTIVE_THRESH_MEAN_C ;
      int type = _threshold_type() == 0 ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
      cv::adaptiveThreshold(image, image, 255, method, type, size, _c_param());
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _method, _threshold_type, _block_size;
  DoubleParameter _c_param;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_INRANGE_H_
