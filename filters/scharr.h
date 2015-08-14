/**
 * \file	Scharr.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_SHCARR_H_
#define VISION_FILTER_SHCARR_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class Scharr : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Scharr(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _convert_to_uchar("Convert_to_uchar", true, parameters_),
        _delta("Delta", 0, 0, 255, parameters_),
        _scale("Scale", 1, 0, 255, parameters_),
        _use_pixel_intensity_correction("use_pixel_intensity_correction", false,
                                        parameters_),
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

}  // namespace vision_filter

//==============================================================================
// U N I T   T E S T S

#include <TCUnitTest.h>
#include <lib_vision/algorithm/general_function.h>

TC_DEFINE_UNIT_TEST(ScharrTestUT) {
  cv::Mat original = cv::imread("/home/jeremie/Videos/originalImage.jpeg");
  cv::cvtColor(original, original, CV_BGR2GRAY);
  cv::Mat scharrX, scharrY;
  cv::Scharr(original, scharrX, CV_32F, 1, 0, 1, 0, cv::BORDER_REPLICATE);
  cv::Scharr(original, scharrY, CV_32F, 0, 1, 1, 0, cv::BORDER_REPLICATE);
  cv::Mat scharrAll;
  cv::addWeighted(scharrX, 0.5, scharrY, 0.5, 0, scharrAll, CV_32FC1);
  cv::Mat scharrCorrected = cv::Mat::zeros(scharrAll.size(), CV_32FC1);
  for (int y = 0; y < scharrCorrected.rows; y++) {
    float *ptrOut = scharrCorrected.ptr<float>(y);
    float *ptrIn = scharrAll.ptr<float>(y);
    for (int x = 0; x < scharrCorrected.cols; x++) {
      ptrOut[x] = pow(fabs(ptrIn[x]), 0.5);
      // ptrOut[x] = log(fabs(ptrIn[x]),0.5);
    }
  }
  cv::Mat finalOut;
  cv::convertScaleAbs(scharrCorrected, finalOut);
  cv::Mat abs_x, abs_y, ucharOutOriginal;
  cv::convertScaleAbs(scharrX, abs_x);
  cv::convertScaleAbs(scharrY, abs_y);
  cv::addWeighted(abs_x, 0.5, abs_y, 0.5, 0, ucharOutOriginal, CV_8UC1);

  cv::imshow("finalresult", finalOut);
  cv::imshow("Original", ucharOutOriginal);
  cv::waitKey(-1);

  return true;
}
TC_END_UNIT_TEST(ScharrTestUT);

#endif  // VISION_FILTER_SHCARR_H_
