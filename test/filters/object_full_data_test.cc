/**
 * \file	object_full_data_test.cc
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

#include <gtest/gtest.h>
#include <provider_vision/algorithm/object_full_data.h>

using namespace provider_vision;

TEST(FullData, AllTest) {
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
  objectVec.push_back(std::make_shared<ObjectFullData>(
      originalImage, binaryImage, contours[0]));
  objectVec.push_back(std::make_shared<ObjectFullData>(
      originalImage, binaryImage, contours[1]));
  objectVec.push_back(std::make_shared<ObjectFullData>(
      originalImage, binaryImage, contours[2]));

  // NO further testing since everything is tested in the parent classes.
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
