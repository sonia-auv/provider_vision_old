/**
 * \file	blurr.h
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

#ifndef LIB_VISION_FILTERS_BLURR_H_
#define LIB_VISION_FILTERS_BLURR_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

// see http://docs.opencv.org/modules/imgproc/doc/filtering.html
// for more detail on how and why
//
// This is a program to execute image filter other than erode, dilate and
// morphologicalEx. Those are more blur function than pixelizer
// settings are for the differents type of filters, and does not apply to all
class Blurr : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Blurr>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Blurr(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _type("Type", 2, 0, 3, &parameters_,
              "1=Blur, 2=GaussianBlur, 3=MedianBlur"),
        _kernel_size("Kernel_size", 1, 0, 35, &parameters_),
        _anchor(-1, -1) {
    setName("Blurr");
  }

  virtual ~Blurr() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::Size2i kernelSize((int)_kernel_size() * 2 + 1,
                            (int)(_kernel_size() * 2 + 1));
      switch (_type()) {
        // Could be optimized via function pointer maybe?
        case 0:
          break;
        case 1:
          cv::blur(image, image, kernelSize, _anchor);
          break;
        case 2:
          cv::GaussianBlur(image, image, kernelSize, 0, 0);
          break;
        case 3:
          cv::medianBlur(image, image, _kernel_size() * 2 + 1);
          break;
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _type, _kernel_size;

  const cv::Point _anchor;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_BLURR_H_
