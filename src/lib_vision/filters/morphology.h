/**
 * \file	morphology.h
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

#ifndef LIB_VISION_FILTERS_MORPHOLOGY_H_
#define LIB_VISION_FILTERS_MORPHOLOGY_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class Morphology : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Morphology>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Morphology(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _morph_type("Morphology_type", 0, 0, 4, &parameters_,
                    "0=Gradient, 1=TopHat, 2=BlackHat, 3=Opening, 4=Closing"),
        _kernel_type("Kernel_type", 0, 0, 2, &parameters_,
                     "0=Rect, 1=Elipse, 2=Cross"),
        _iteration("Iteration", 1, 1, 20, &parameters_),
        _kernel_size("Kernel_size", 1, 1, 40, &parameters_),
        _anchor(-1, -1) {
    setName("Morphology");
  }

  virtual ~Morphology() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }

      // Kernel selection
      int kernelType;
      switch (_kernel_type()) {
        case 0:
          kernelType = cv::MORPH_RECT;
          break;
        case 1:
          kernelType = cv::MORPH_ELLIPSE;
          break;
        case 2:
          kernelType = cv::MORPH_CROSS;
          break;
        default:
          kernelType = cv::MORPH_RECT;
          break;
      }

      // Creating the kernel
      cv::Mat kernel = cv::getStructuringElement(
          kernelType, cv::Size(_kernel_size() * 2 + 1, _kernel_size() * 2 + 1),
          _anchor);

      // Selecting with _morph_type wich operation to use
      switch (_morph_type()) {
        case 0:
          cv::morphologyEx(image, image, cv::MORPH_GRADIENT, kernel, _anchor,
                           _iteration(), CV_8U);
          break;
        case 1:
          cv::morphologyEx(image, image, cv::MORPH_TOPHAT, kernel, _anchor,
                           _iteration(), CV_8U);
          break;
        case 2:
          cv::morphologyEx(image, image, cv::MORPH_BLACKHAT, kernel, _anchor,
                           _iteration(), CV_8U);
          break;
        case 3:
          cv::morphologyEx(image, image, cv::MORPH_OPEN, kernel, _anchor,
                           _iteration(), CV_8U);
          break;
        case 4:
          cv::morphologyEx(image, image, cv::MORPH_CLOSE, kernel, _anchor,
                           _iteration(), CV_8U);
          break;
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _morph_type, _kernel_type, _iteration, _kernel_size;
  const cv::Point _anchor;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_MORPHOLOGY_H_
