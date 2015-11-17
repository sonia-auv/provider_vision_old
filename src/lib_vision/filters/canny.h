/**
 * \file	canny.h
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

#ifndef LIB_VISION_FILTERS_CANNY_H_
#define LIB_VISION_FILTERS_CANNY_H_

#include <lib_vision/filter.h>

namespace lib_vision {

class Canny : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Canny(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _l2_gradiant("l2_gradient", false, parameters_),
        _thresh_one("thres_one", 100, 0, 255, parameters_),
        _thresh_two("thres_two", 200, 0, 255, parameters_),
        _aperture_size("Aperture_size", 3, 0, 20, parameters_) {
    setName("Canny");
  }

  virtual ~Canny() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      cv::Canny(image, image, _thresh_one(), _thresh_two(),
                _aperture_size() * 2 + 1, _l2_gradiant());
    }
  }

 private:
  // Params
  BooleanParameter _enable, _l2_gradiant;
  IntegerParameter _thresh_one, _thresh_two, _aperture_size;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_SOBEL_H_
