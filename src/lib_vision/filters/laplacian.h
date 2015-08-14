/**
 * \file  Sobel.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date  14/12/2014
 * \copyright Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_LAPLACIAN_H_
#define VISION_FILTER_LAPLACIAN_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class Laplacian : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Laplacian(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _convert_to_uchar("Convert_to_uchar", true, parameters_),
        _delta("Delta", 0, 0, 255, parameters_),
        _scale("Scale", 1, 0, 255, parameters_),
        _size("Size", 2, 1, 20, parameters_) {
    setName("Laplacian");
  }

  virtual ~Laplacian() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      int size = _size() * 2 + 1;

      if (_convert_to_uchar()) {
        cv::Laplacian(image, image, CV_8U, size, _scale(), _delta(),
                      cv::BORDER_DEFAULT);
      } else {
        cv::Laplacian(image, image, CV_32F, size, _scale(), _delta(),
                      cv::BORDER_DEFAULT);
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable, _convert_to_uchar;
  DoubleParameter _delta, _scale;
  IntegerParameter _size;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_SOBEL_H_
