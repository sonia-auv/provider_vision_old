/**
 * \file	object_ranker.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_ranker.h>

//-----------------------------------------------------------------------------
//
void ObjectRanker::RankByArea(ObjectFullData::FullObjectPtrVec objects) {
  std::sort(objects.begin(), objects.end(), ObjectRanker::AreaSortFunction);
  for (int i = 0, size = objects.size(); i < size; i++) {
    if (objects[i].IsNotNull()) {
      objects[i]->SetAreaRank((float(size - i)) / float(size));
    }
  }
}

//-----------------------------------------------------------------------------
//
void ObjectRanker::RankByLength(ObjectFullData::FullObjectPtrVec objects) {
  std::sort(objects.begin(), objects.end(), ObjectRanker::LengthSortFunction);
  for (int i = 0, size = objects.size(); i < size; i++) {
    if (objects[i].IsNotNull()) {
      objects[i]->SetLengthRank(float((size - i)) / float(size));
    }
  }
}

//=============================================================================
//=============================================================================
//=============================================================================
//=========================== UNIT TEST AREA ==================================
//=============================================================================
//=============================================================================
//=============================================================================
#include <TCUnitTest.h>

TC_DEFINE_UNIT_TEST(ObjectRankerUT) {
  printf("Starting unit test on ObjectRanker");
  printf("Starting unit test on ObjectBasicData\n");

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

  tmpContour.push_back(cv::Point(20, 20));
  tmpContour.push_back(cv::Point(20, 100));
  tmpContour.push_back(cv::Point(21, 100));
  tmpContour.push_back(cv::Point(21, 20));
  contours.push_back(tmpContour);
  cv::drawContours(originalImage, contours, -1, CV_RGB(255, 0, 255), -1);

  cv::cvtColor(originalImage, binaryImage, CV_BGR2GRAY);
  //	cv::imshow("test", originalImage);
  //	cv::waitKey(-1);

  ObjectFullData::FullObjectPtrVec tmp;
  tmp.push_back(ObjectFullData::Ptr(
      new ObjectFullData(originalImage, binaryImage, contours[0])));
  tmp.push_back(ObjectFullData::Ptr(
      new ObjectFullData(originalImage, binaryImage, contours[1])));

  ObjectRanker ranker;
  ranker.RankByArea(tmp);
  ranker.RankByLength(tmp);

  // So the contours were added in order of area, so the smallest to the biggest
  // however, the second contour is longer than the third one, so the ranking
  // should be like this loop.
  // Remember that when we sort the vector, we alter the order of the vector
  // so they are not in the order they were added.
  for (int i = 0; i < tmp.size(); i++) {
    printf("AreaRank Area LengthRank Length %f %f %f %f\n",
           tmp[i]->GetAreaRank(), tmp[i]->GetArea(), tmp[i]->GetLengthRank(),
           tmp[i]->GetLength());
  }
  printf("System all clear and good to go");
  return true;
}
TC_END_UNIT_TEST(ObjectRankerUT);
