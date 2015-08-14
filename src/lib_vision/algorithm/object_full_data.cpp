/**
 * \file	object_full_data.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_full_data.h>

ObjectFullData::ObjectFullData(const cv::Mat &originalImage,
                               const cv::Mat &binaryImage,
                               const contour_t &contour)
    : ObjectBasicData(originalImage, binaryImage, contour) {}

//=============================================================================
//=============================================================================
//=============================================================================
//=========================== UNIT TEST AREA ==================================
//=============================================================================
//=============================================================================
//=============================================================================
#include <TCUnitTest.h>

TC_DEFINE_UNIT_TEST(ObjectFullDataUT) {
  printf("Starting unit test on ObjectFullData");
  contourList_t contours;
  cv::Mat binaryImage(1000, 1000, CV_8UC1, cv::Scalar::all(0));
  cv::Mat originalImage(1000, 1000, CV_8UC3, cv::Scalar::all(0));

  // square contour 100 of area, 1 of ratio
  // 100 filled, convexity 0 , circularity ?
  // last in area rank and length rank
  std::vector<cv::Point> tmpContour;
  tmpContour.push_back(cv::Point(0, 0));
  tmpContour.push_back(cv::Point(0, 10));
  tmpContour.push_back(cv::Point(10, 10));
  tmpContour.push_back(cv::Point(10, 0));
  contours.push_back(tmpContour);
  tmpContour.clear();

  // M like contour.
  tmpContour.push_back(cv::Point(100, 100));
  tmpContour.push_back(cv::Point(200, 100));
  tmpContour.push_back(cv::Point(150, 300));
  tmpContour.push_back(cv::Point(200, 500));
  tmpContour.push_back(cv::Point(100, 500));
  contours.push_back(tmpContour);
  tmpContour.clear();

  tmpContour.push_back(cv::Point(400, 700));
  tmpContour.push_back(cv::Point(750, 700));
  tmpContour.push_back(cv::Point(750, 800));
  tmpContour.push_back(cv::Point(400, 800));
  contours.push_back(tmpContour);
  cv::drawContours(originalImage, contours, -1, CV_RGB(255, 0, 255), -1);
  cv::cvtColor(originalImage, binaryImage, CV_BGR2GRAY);

  std::vector<ObjectFullData::Ptr> objectVec;
  objectVec.push_back(
      new ObjectFullData(originalImage, binaryImage, contours[0]));
  objectVec.push_back(
      new ObjectFullData(originalImage, binaryImage, contours[1]));
  objectVec.push_back(
      new ObjectFullData(originalImage, binaryImage, contours[2]));

  // NO further testing since everything is tested in the parent classes.

  printf("System all clear and good to go");
  return true;
}
TC_END_UNIT_TEST(ObjectFullDataUT);
