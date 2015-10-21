/**
 * \file	object_basic_data_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/object_basic_data.h>
#include <lib_vision/algorithm/general_function.h>
#include "lib_atlas/config.h"

bool CompareMoments(cv::Moments moment1, cv::Moments moment2) {
  return moment1.m00 == moment2.m00 && moment1.m01 == moment2.m01 &&
         moment1.m02 == moment2.m02 && moment1.m03 == moment2.m03 &&
         moment1.m10 == moment2.m10 && moment1.m20 == moment2.m20 &&
         moment1.m30 == moment2.m30;
}

TEST(BasicData, AllTest) {
  cv::Mat unitTestImage, binaryImage;
  std::string pathToFile = atlas::kWorkspaceRoot + "src/lib_vision/test/BasicObjectUnitTest.png";
  unitTestImage = cv::imread(pathToFile);
  ASSERT_TRUE(!unitTestImage.empty());

  if (unitTestImage.channels() != 1) {
    cv::cvtColor(unitTestImage, binaryImage, CV_BGR2GRAY);
  }
  contourList_t contours;
  hierachy_t hierachy;
  retrieveAllContours(binaryImage, contours);

  std::vector<ObjectBasicData> objectData;

  for (int i = 0; i < contours.size(); i++) {
    objectData.push_back(
        ObjectBasicData(unitTestImage, binaryImage, contours[i]));
  }

  ASSERT_TRUE(objectData.size() > 0);
  unitTestImage = cv::Mat::zeros(100, 100, CV_8UC3);
  for (int i = 0; i < objectData.size(); i++) {
    cv::Mat original_image = objectData[i].GetOriginalImage();
    ASSERT_TRUE(original_image.cols != 100 &&
                original_image.rows != 100);

    float area = objectData[i].GetArea();
    ASSERT_TRUE(area > 0.0f);

    float convexityArea = objectData[i].GetConvexHullArea();
    ASSERT_TRUE(convexityArea > 0.0f);

    float circumference = objectData[i].GetCircumference();
    ASSERT_TRUE(circumference > 0.0f);

    cv::Moments moments = objectData[i].GetMoments(true);
    ASSERT_TRUE(CompareMoments(objectData[i].GetMoments(false), moments));

    RotRect rrect = objectData[i].GetRotatedRect();
    ASSERT_TRUE(rrect == objectData[i].GetRotatedRect());

    cv::Rect uprightRect = objectData[i].GetUprightRect();
    ASSERT_TRUE(uprightRect == objectData[i].GetUprightRect());

    cv::Mat plane = objectData[i].GetPlanes(ObjectBasicData::BLUE_PLANE);
    ASSERT_TRUE(!plane.empty());
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
