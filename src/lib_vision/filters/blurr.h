/**
 * \file	Blurr.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_BLURR_H_
#define VISION_FILTER_BLURR_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

// see http://docs.opencv.org/modules/imgproc/doc/filtering.html
// for more detail on how and why
//
// This is a program to execute image filter other than erode, dilate and
// morphologicalEx. Those are more blur function than pixelizer
// settings are for the differents type of filters, and does not apply to all
class Blurr : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Blurr(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _type("Type", 2, 0, 3, parameters_,
              "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
        _kernel_size("Kernel_size", 1, 0, 35, parameters_),
        _anchor(-1, -1) {
    setName("Blurr");
  }

  virtual ~Blurr() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::Size2i kernelSize((int)_kernel_size() * 2 + 1,
                            (int)(_kernel_size() * 2 + 1));
      switch (_type()) {
        // Could be optimized via function pointer maybe?
        case 0:
          break;
        case 1:
          cv::blur(image, image, kernelSize, _anchor);
          break;
        case 2:
          cv::GaussianBlur(image, image, kernelSize, 0, 0);
          break;
        case 3:
          cv::medianBlur(image, image, _kernel_size() * 2 + 1);
          break;
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _type, _kernel_size;

  const cv::Point _anchor;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_BLURR_H_
