/**
 * \file	adaptative_threshold.h
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

#ifndef LIB_VISION_FILTERS_ADAPTIVE_THRESHOLD_H_
#define LIB_VISION_FILTERS_ADAPTIVE_THRESHOLD_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class AdaptiveThreshold : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<AdaptiveThreshold>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit AdaptiveThreshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _method("Method", 0, 0, 1, &parameters_, "0=Gaussian 1=Mean"),
        _threshold_type("Threshold_type", 0, 0, 1, &parameters_,
                        "0=BIN, 1=BIN_INV"),
        _block_size("Size", 1, 1, 40, &parameters_),
        _c_param("C_param", 0.0f, -255.0f, 255.0f, &parameters_) {
    SetName("AdaptiveThreshold");
  }

  virtual ~AdaptiveThreshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      int size = _block_size() * 2 + 1;
      int method = _method() == 0 ? cv::ADAPTIVE_THRESH_GAUSSIAN_C
                                  : cv::ADAPTIVE_THRESH_MEAN_C;
      int type =
          _threshold_type() == 0 ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
      cv::adaptiveThreshold(image, image, 255, method, type, size, _c_param());
    }
  }

 private:
  // Params
  Parameter<bool> _enable;
  RangedParameter<int> _method, _threshold_type, _block_size;
  RangedParameter<double> _c_param;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_INRANGE_H_
