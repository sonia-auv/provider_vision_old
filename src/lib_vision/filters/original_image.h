/**
 * \file	OriginalImage.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_ORIGINAL_IMAGE_H_
#define VISION_FILTER_ORIGINAL_IMAGE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class OriginalImage : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit OriginalImage(const GlobalParamHandler &globalParams)
      : Filter(globalParams), _enable("Enable", false, parameters_) {
    setName("OriginalImage");
  }

  virtual ~OriginalImage() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      image = global_params_.getOriginalImage();
    }
  }

 private:
  // Params
  BooleanParameter _enable;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_ORIGINAL_IMAGE_H_
