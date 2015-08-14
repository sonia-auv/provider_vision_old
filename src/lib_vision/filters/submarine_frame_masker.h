/**
 * \file	SubmarineFrameMasker.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_SUB_FRAME_MASK_H_
#define VISION_FILTER_SUB_FRAME_MASK_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class SubmarineFrameMasker : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit SubmarineFrameMasker(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _rotate_type("Rotation_type", 0, 0, 3, parameters_,
                     "Rotate type: 0=NONE, 1=x axis, 2=y axis, 3=all axis"),
        _prev_rot_value(0) {
    setName("SubmarineFrameMasker");
    std::string mask_name =
        std::string(getenv("SONIA_WORKSPACE_ROOT")) +
        std::string("/ros/src/vision_server/config/bottom_mask.jpg");
    _bottom_mask = cv::imread(mask_name, CV_LOAD_IMAGE_GRAYSCALE);
  }

  virtual ~SubmarineFrameMasker() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (_prev_rot_value != _rotate_type()) {
        _prev_rot_value = _rotate_type();
        switch (_rotate_type()) {
          case 1:
            cv::flip(_bottom_mask, _bottom_mask, 0);
            break;
          case 2:
            cv::flip(_bottom_mask, _bottom_mask, 1);
            break;
          case 3:
            cv::flip(_bottom_mask, _bottom_mask, -1);
            break;
        }
      }
      if (image.size() == _bottom_mask.size())
        cv::bitwise_and(image, _bottom_mask, image);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _rotate_type;
  cv::Mat _bottom_mask;
  int _prev_rot_value;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_SUB_FRAME_MASK_H_
