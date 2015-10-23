/**
 * \file	object_ranker_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/object_ranker.h>

TEST(Ranker, AllTest) {
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
  tmp.push_back(
      ObjectFullData::Ptr(std::make_shared<ObjectFullData>(
          originalImage, binaryImage, contours[0])));
  tmp.push_back(
      ObjectFullData::Ptr(std::make_shared<ObjectFullData>(
          originalImage, binaryImage, contours[1])));

  ObjectRanker ranker;
  ranker.RankByArea(tmp);
  ranker.RankByLength(tmp);

  // So the contours were added in order of area, so the smallest to the biggest
  // however, the second contour is longer than the third one, so the ranking
  // should be like this loop.
  // Remember that when we sort the vector, we alter the order of the vector
  // so they are not in the order they were added.
//  for (int i = 0; i < tmp.size(); i++) {
//    printf("AreaRank Area LengthRank Length %f %f %f %f\n",
//           tmp[i]->GetAreaRank(), tmp[i]->GetArea(), tmp[i]->GetLengthRank(),
//           tmp[i]->GetLength());
//  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
