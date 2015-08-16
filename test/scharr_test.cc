/**
 * \file	scharr_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/filters/scharr.h>

TEST(AITrainer, AllTest) {
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
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
