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

#include <lib_vision/filter.h>

namespace lib_vision {

class Threshold : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Threshold>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Threshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _type("Threshold_type", 1, 0, 5, parameters_,
              "0=BIN, 1=BIN_INV, 2=TRUNC, 3=TOZERO, 4=TOZERO_INV 5=OTSU"),
        _max("Max_value", 100, 0, 255, parameters_) {
    setName("Threshold");
  }

  virtual ~Threshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      if (image.depth() != CV_8U) {
        image.convertTo(image, CV_8U);
      }

      int threshold_type = CV_THRESH_BINARY;
      switch (_type()) {
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
  // Params
  BooleanParameter _enable;
  IntegerParameter _type, _max;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_THRESHOLD_H_
