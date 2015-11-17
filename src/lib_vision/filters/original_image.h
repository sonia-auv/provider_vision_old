/**
 * \file	original_image.h
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

#ifndef LIB_VISION_FILTERS_ORIGINAL_IMAGE_H_
#define LIB_VISION_FILTERS_ORIGINAL_IMAGE_H_

#include <lib_vision/filter.h>

namespace lib_vision {

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

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_ORIGINAL_IMAGE_H_
