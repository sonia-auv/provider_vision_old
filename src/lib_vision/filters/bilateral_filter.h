/**
 * \file	bilateral_filter.h
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

#ifndef LIB_VISION_FILTERS_BILATERAL_FILTER_H_
#define LIB_VISION_FILTERS_BILATERAL_FILTER_H_

#include <lib_vision/filter.h>

namespace lib_vision {

/**
 * The filter bilateral aims to blurr an image without loosing edge sharpness.
 * This act like a proxy for the OpenCv bilateralFilter method.
 */
class BilateralFilter : public Filter {
 public:
  //============================================================================
  // P U B L I C   C / D T O R

  explicit BilateralFilter(const GlobalParamHandler &globalParams) noexcept
      : Filter(globalParams),
        enable_("Enable", false, parameters_),
        diameter_("Diameter", -100, 0, 100, parameters_),
        sigma_color_("Sigm_color", 0, 0, 300, parameters_),
        sigma_space_("Sigma_space", 0, 0, 300, parameters_) {
    setName("BilateralFilter");
  }

  virtual ~BilateralFilter() noexcept {}

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Override the execute function from the Filter class.
   * This is the function that is going to be called for processing the image.
   * This takes an image as a parameter and modify it with the filtered image.
   *
   * \param image The image to process.
   */
  void execute(cv::Mat &image) override {
    if (enable_()) {
      cv::Mat blurred;
      cv::bilateralFilter(image, blurred, diameter_.getValue(),
                          sigma_color_.getValue(), sigma_space_.getValue());

      blurred.copyTo(image);
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  /**
   * State if the filter is enabled or not.
   * This is being used by the vision server for calling the filter in the
   * filterchain.
   */
  BooleanParameter enable_;

  /**
   * From the OpenCV definition:
   * Diameter of each pixel neighborhood that is used during filtering.
   * If it is non-positive, it is computed from sigmaSpace .
   */
  IntegerParameter diameter_;

  /**
   * From the OpenCV definition:
   * Filter sigma in the color space. A larger value of the parameter means
   * that farther colors within the pixel neighborhood (see sigmaSpace )
   * will be mixed together, resulting in larger areas of semi-equal color.
   */
  IntegerParameter sigma_color_;

  /**
   * From the OpenCV definition:
   * Filter sigma in the coordinate space. A larger value of the parameter
   * means that farther pixels will influence each other as long as their
   * colors are close enough (see sigmaColor ). When d>0 , it specifies
   * the neighborhood size regardless of sigmaSpace . Otherwise, d is
   * proportional to sigmaSpace .
   */
  IntegerParameter sigma_space_;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_BILATERAL_FILTER_H_
