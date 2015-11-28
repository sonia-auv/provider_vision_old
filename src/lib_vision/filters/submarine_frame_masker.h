/**
 * \file	submarine_frame_masker.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Pierluc Bédard <pierlucbed@gmail.com>
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIB_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_
#define LIB_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_

#include <lib_vision/filter.h>

namespace lib_vision {

class SubmarineFrameMasker : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<SubmarineFrameMasker>;

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

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_SUBMARINE_FRAME_MASKER_H_
