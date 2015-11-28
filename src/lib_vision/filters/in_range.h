/**
 * \file	in_range.h
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

#ifndef LIB_VISION_FILTERS_IN_RANGE_FILTER_H_
#define LIB_VISION_FILTERS_IN_RANGE_FILTER_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

/**
 * The filter inRange check if array elements lie between certain value of HSV
 * and Luv. If it does, value of pixel is set to = 1.
 * If not, value of pixel = 0.
 * In this case, this filter is a binarizer and will output a black and white
 * image
 */
class InRange : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<InRange>;

  //============================================================================
  // P U B L I C   C / D T O R

  explicit InRange(const GlobalParamHandler &globalParams) noexcept
      : Filter(globalParams),
        enable_("Enable", false, parameters_),
        lower_hue_("HSVLowH", 0, 0, 255, parameters_),
        upper_hue_("HSVHighH", 255, 0, 255, parameters_),
        lower_saturation_("HSVLowS", 0, 0, 255, parameters_),
        upper_saturation_("HSVHighS", 255, 0, 255, parameters_),
        lower_value_("HSVLowV", 0, 0, 255, parameters_),
        upper_value_("HSVHighV", 255, 0, 255, parameters_),
        lower_lightness_("LUVlowL", 0, 0, 255, parameters_),
        upper_lightness_("LUVhighL", 255, 0, 255, parameters_),
        lower_u_("LUVlowU", 0, 0, 255, parameters_),
        upper_u_("LUVhighU", 255, 0, 255, parameters_),
        lower_v_("LUVlowV", 0, 0, 255, parameters_),
        upper_v_("LUVhighV", 255, 0, 255, parameters_) {
    setName("InRange");
  }

  virtual ~InRange() noexcept {}

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Overrides the execute function from the Filter class.
   * This is the function that is going to be called for processing the image.
   * This takes an image as a parameter and modify it with the filtered image.
   *
   * \param image The image to process.
   */
  void execute(cv::Mat &image) override {
    if (enable_.getValue()) {
      cv::Mat hsv;
      cv::Mat luv;

      cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV_FULL);
      cv::inRange(
          hsv, cv::Scalar(lower_hue_.getValue(), lower_saturation_.getValue(),
                          lower_value_.getValue()),
          cv::Scalar(upper_hue_.getValue(), upper_saturation_.getValue(),
                     upper_value_.getValue()),
          hsv);

      cv::cvtColor(image, luv, cv::COLOR_RGB2Luv);
      cv::inRange(luv, cv::Scalar(lower_lightness_.getValue(),
                                  lower_u_.getValue(), lower_v_.getValue()),
                  cv::Scalar(upper_lightness_.getValue(), upper_u_.getValue(),
                             upper_v_.getValue()),
                  luv);
      cv::bitwise_and(hsv, luv, image);
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

  /** Inclusive Hue lower boundary. */
  IntegerParameter lower_hue_;

  /**  Inclusive Hue upper boundary. */
  IntegerParameter upper_hue_;

  /** Inclusive Saturation lower boundary. */
  IntegerParameter lower_saturation_;

  /** Inclusive Saturation upper boundary. */
  IntegerParameter upper_saturation_;

  /** Inclusive Value lower boundary. */
  IntegerParameter lower_value_;

  /** Inclusive Value upper boundary. */
  IntegerParameter upper_value_;

  /** Inclusive Lightness lower boundary. */
  IntegerParameter lower_lightness_;

  /** Inclusive Lightness upper boundary. */
  IntegerParameter upper_lightness_;

  /** Inclusive u lower boundary. */
  IntegerParameter lower_u_;

  /** Inclusive u upper boundary. */
  IntegerParameter upper_u_;

  /** Inclusive v lower boundary. */
  IntegerParameter lower_v_;

  /** Inclusive v upper boundary. */
  IntegerParameter upper_v_;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_IN_RANGE_FILTER_H_
