/**
 * \file	ai_trainer_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "lib_vision/algorithm/contour_list.h"
#include "lib_vision/algorithm/object_feature_factory.h"
#include "lib_vision/algorithm/object_full_data.h"
#include "lib_atlas/config.h"

TEST(FeatureFactory, AllTest) {

  cv::Mat original_image =
      cv::imread(atlas::kWorkspaceRoot + "src/lib_vision/test/img/FeatureFactoryTestImage.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat binary_image;
  cv::cvtColor(original_image, binary_image, CV_RGB2GRAY);
  cv::Mat debug = cv::Mat::zeros(original_image.rows, original_image.cols, CV_8UC3);
  ContourList list(binary_image, ContourList::OUTER);

  ASSERT_EQ(list.size(), 4);

  ObjectFeatureFactory feat_factory(3);
  ObjectFullData::FullObjectPtrVec obj_vec;
  for( auto &contour : list.GetAsPoint())
  {
    obj_vec.push_back(std::make_shared<ObjectFullData>(original_image, binary_image, contour));
  }
  feat_factory.ComputeAllFeature(obj_vec);

  // So the order is:
  //  - Crescent. Should have convexity index very low. percent filled 1.
  //  - Full circle. Circularity index very High, ratio too. Convexity 0, percente filled 1
  //  - Small square. Ratio of 1, convexity 0, percent filled 1
  //  - Big square. Ratio around 8, percented filled near 0, convexity 1,

  for(auto &tmp : obj_vec)
  {
    std::cout << "New object" << std::endl;
    std::cout << "Ratio: " << tmp->GetRatio() << std::endl;
    std::cout << "Convexity: " << tmp->GetConvexity() << std::endl;
    std::cout << "Percent filled: " << tmp->GetPercentFilled() << std::endl;
    std::cout << "Circularity: " << tmp->GetCircularity() << std::endl;

    std::cout << "Hue: " << tmp->GetHueMean() << std::endl;
    std::cout << "Sat: " << tmp->GetSatMean() << std::endl;
    std::cout << "Int: " << tmp->GetIntensityMean() << std::endl;

    std::cout << "Blue: " << tmp->GetBlueMean() << std::endl;
    std::cout << "Red: " << tmp->GetRedMean() << std::endl;
    std::cout << "Green: " << tmp->GetGreenMean() << std::endl;

    std::cout << "Gray: " << tmp->GetGrayMean() << std::endl;

    std::cout << std::endl;
  }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
