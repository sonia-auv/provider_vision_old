/**
 * \file	threshold.h
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

#ifndef LIB_VISION_FILTERS_THRESHOLD_H_
#define LIB_VISION_FILTERS_THRESHOLD_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class Threshold : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Threshold>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Threshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        type_("Threshold_type", 1, 0, 5, &parameters_,
              "0=BIN, 1=BIN_INV, 2=TRUNC, 3=TOZERO, 4=TOZERO_INV 5=OTSU"),
        _max("Max_value", 100, 0, 255, &parameters_) {
    SetName("Threshold");
  }

  virtual ~Threshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      if (image.depth() != CV_8U) {
        image.convertTo(image, CV_8U);
      }

      int threshold_type = CV_THRESH_BINARY;
      switch (type_()) {
        case 0:
          threshold_type = CV_THRESH_BINARY;
          break;
        case 1:
          threshold_type = CV_THRESH_BINARY_INV;
          break;
        case 2:
          threshold_type = CV_THRESH_TRUNC;
          break;
        case 3:
          threshold_type = CV_THRESH_TOZERO;
          break;
        case 4:
          threshold_type = CV_THRESH_TOZERO_INV;
          break;
        case 5:
          threshold_type = CV_THRESH_OTSU;
          break;
        default:
          threshold_type = CV_THRESH_BINARY;
          break;
      }
      cv::threshold(image, image, _max(), 255, threshold_type);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> enable_;
  RangedParameter<int> type_, _max;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_THRESHOLD_H_
