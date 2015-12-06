/**
 * \file	schar_adding.h
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

#ifndef LIB_VISION_FILTERS_SCHARR_ADDING_H_
#define LIB_VISION_FILTERS_SCHARR_ADDING_H_

#include <memory>
#include <lib_vision/filter.h>
#include <lib_vision/algorithm/general_function.h>

namespace lib_vision {

class ScharrAdding : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ScharrAdding>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ScharrAdding(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _run_small_image("Run_small_image", true, &parameters_,
                         "Resize image to run on smaller image"),
        _convert_to_uchar("Convert_to_uchar", false, &parameters_),
        _delta("Delta", 0, 0, 255, &parameters_),
        _scale("Scale", 1, 0, 255, &parameters_),
        _mean_multiplier("Mean_multiplier", 1.0f, 0.0f, 10.0f, &parameters_),
        _plane_blue("Blue", false, &parameters_),
        _plane_green("Green", false, &parameters_),
        _plane_red("Red", false, &parameters_),
        _plane_hue("Hue", false, &parameters_),
        _plane_saturation("Saturation", false, &parameters_),
        _plane_intensity("Intensity", false, &parameters_),
        _plane_gray("Gray", false, &parameters_) {
    SetName("ScharrAdding");
  }

  virtual ~ScharrAdding() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() != 3) return;
      if (_run_small_image()) {
        cv::resize(image, image, cv::Size(image.cols / 2, image.rows / 2));
      }

      std::vector<cv::Mat> colorPlanes = getColorPlanes(image);
      cv::Mat sum = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);

      if (_plane_blue()) cv::add(calcScharr(colorPlanes[0]), sum, sum);
      if (_plane_green()) cv::add(calcScharr(colorPlanes[1]), sum, sum);
      if (_plane_red()) cv::add(calcScharr(colorPlanes[2]), sum, sum);
      if (_plane_hue()) cv::add(calcScharr(colorPlanes[3]), sum, sum);
      if (_plane_saturation()) cv::add(calcScharr(colorPlanes[4]), sum, sum);
      if (_plane_intensity()) cv::add(calcScharr(colorPlanes[5]), sum, sum);
      if (_plane_gray()) cv::add(calcScharr(colorPlanes[6]), sum, sum);

      sum.copyTo(image);
      if (_run_small_image()) {
        cv::resize(image, image, cv::Size(image.cols * 2, image.rows * 2));
      }

      if (_convert_to_uchar() && image.channels() < 3) {
        cv::cvtColor(image, image, CV_GRAY2BGR);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  cv::Mat calcScharr(const cv::Mat &img) {
    cv::Mat abs_x, scharrX, abs_y, scharrY, diff;

    cv::Scharr(img, scharrX, CV_32F, 1, 0, _scale(), _delta(),
               cv::BORDER_REPLICATE);
    cv::Scharr(img, scharrY, CV_32F, 0, 1, _scale(), _delta(),
               cv::BORDER_REPLICATE);
    cv::absdiff(scharrX, 0, scharrX);
    cv::absdiff(scharrY, 0, scharrY);

    cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, diff, CV_32F);

    cv::Scalar mean = cv::mean(diff);
    cv::threshold(diff, diff, (mean[0] * _mean_multiplier()), 0,
                  CV_THRESH_TOZERO);

    return diff;
  }

  //============================================================================
  // P R I V A T E   M E M B E R S

  // _run_small_image accelerate the pipeline by
  // reducing the image size by two (in each direction)
  // so that the scharr computation does not take to much time
  // when multiple images.
  Parameter<bool> _enable, _run_small_image, _convert_to_uchar;
  // _mean_multiplier act as threshold for noise.
  // When set, it remove everything under the mean to keep only
  // proeminent contours.
  RangedParameter<double> _delta, _scale, _mean_multiplier;
  Parameter<bool> _plane_blue, _plane_green, _plane_red;
  Parameter<bool> _plane_hue, _plane_saturation, _plane_intensity;
  Parameter<bool> _plane_gray;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_SCHARR_ADDING_H_
