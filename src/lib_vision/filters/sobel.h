/**
 * \file	Sobel.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_SOBEL_H_
#define VISION_FILTER_SOBEL_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class Sobel : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Sobel(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _convert_to_uchar("Convert_to_uchar", true, parameters_),
        _use_pixel_intensity_correction("use_pixel_intensity_correction", false,
                                        parameters_),
        _delta("Delta", 0, 0, 255, parameters_),
        _scale("Scale", 1, 0, 255, parameters_),
        _power_pixel_correction("pixel_correction_power", 1, -10, 10,
                                parameters_),
        _size("Size", 2, 1, 20, parameters_) {
    setName("Sobel");
  }

  virtual ~Sobel() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      cv::Mat sobelX, sobelY;
      int size = _size() * 2 + 1;
      cv::Sobel(image, sobelX, CV_32F, 1, 0, size, _scale(), _delta(),
                cv::BORDER_DEFAULT);
      cv::Sobel(image, sobelY, CV_32F, 0, 1, size, _scale(), _delta(),
                cv::BORDER_DEFAULT);

      cv::absdiff(sobelX, 0, sobelX);
      cv::absdiff(sobelY, 0, sobelY);
      cv::addWeighted(sobelX, 0.5, sobelY, 0.5, 0, image, CV_32F);

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
  IntegerParameter _size;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_SOBEL_H_
