/**
 * \file	dilate.h
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

#ifndef LIB_VISION_FILTERS_DILATE_H_
#define LIB_VISION_FILTERS_DILATE_H_

#include <lib_vision/filter.h>

namespace lib_vision {

class Dilate : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Dilate>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Dilate(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _use_square_kernel("Square_kernel", true, parameters_),
        _kernel_type("Kernel_type", 0, 0, 2, parameters_),
        _kernel_size_x("Width", 1, 0, 20, parameters_),
        _kernel_size_y("Height", 1, 0, 20, parameters_),
        _iteration("Iteration", 1, 0, 20, parameters_),
        _anchor(-1, -1) {
    setName("Dilate");
  }

  virtual ~Dilate() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      int kernel_type = 0;
      switch (_kernel_type()) {
        case 0:
          kernel_type = cv::MORPH_RECT;
          break;
        case 1:
          kernel_type = cv::MORPH_ELLIPSE;
          break;
        case 2:
          kernel_type = cv::MORPH_CROSS;
          break;
      }

      cv::Size size(_kernel_size_x() * 2 + 1,
                    (_use_square_kernel() ? _kernel_size_x() * 2 + 1
                                          : _kernel_size_y() * 2 + 1));
      cv::Mat kernel = cv::getStructuringElement(kernel_type, size, _anchor);

      cv::dilate(image, image, kernel, _anchor, _iteration());
    }
  }

 private:
  // Params
  BooleanParameter _enable, _use_square_kernel;
  IntegerParameter _kernel_type;
  IntegerParameter _kernel_size_x, _kernel_size_y;
  IntegerParameter _iteration;

  const cv::Point _anchor;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_DILATE_H_
