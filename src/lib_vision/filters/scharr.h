/**
 * \file	scharr.h
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

#ifndef LIB_VISION_FILTERS_SHCARR_H_
#define LIB_VISION_FILTERS_SHCARR_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class Scharr : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Scharr>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Scharr(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _convert_to_uchar("Convert_to_uchar", true, parameters_),
        _use_pixel_intensity_correction("use_pixel_intensity_correction", false,
                                        parameters_),
        _delta("Delta", 0, 0, 255, parameters_),
        _scale("Scale", 1, 0, 255, parameters_),
        _power_pixel_correction("pixel_correction_power", 1, -10, 10,
                                parameters_) {
    setName("Scharr");
  }

  virtual ~Scharr() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      cv::Mat scharrX, scharrY;
      cv::Scharr(image, scharrX, CV_32F, 1, 0, _scale(), _delta(),
                 cv::BORDER_REPLICATE);
      cv::Scharr(image, scharrY, CV_32F, 0, 1, _scale(), _delta(),
                 cv::BORDER_REPLICATE);
      cv::absdiff(scharrX, 0, scharrX);
      cv::absdiff(scharrY, 0, scharrY);

      cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, image, CV_32F);

      if (_use_pixel_intensity_correction()) {
        for (int y = 0; y < image.rows; y++) {
          float *ptr = image.ptr<float>(y);
          for (int x = 0; x < image.cols; x++) {
            ptr[x] = pow(ptr[x], _power_pixel_correction());
          }
        }
      }

      if (_convert_to_uchar()) {
        cv::convertScaleAbs(image, image);
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable, _convert_to_uchar, _use_pixel_intensity_correction;
  DoubleParameter _delta, _scale, _power_pixel_correction;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_SHCARR_H_
