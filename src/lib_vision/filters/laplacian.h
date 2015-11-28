/**
 * \file	laplacian.h
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

#ifndef LIB_VISION_FILTERS_LAPLACIAN_H_
#define LIB_VISION_FILTERS_LAPLACIAN_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class Laplacian : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Laplacian>;

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

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_SOBEL_H_
