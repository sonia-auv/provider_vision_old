/**
 * \file	feature_factory_test.cc
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
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "lib_vision/algorithm/contour_list.h"
#include "lib_vision/algorithm/object_feature_factory.h"
#include "lib_vision/algorithm/object_full_data.h"
#include "lib_atlas/config.h"

using namespace lib_vision;

TEST(FeatureFactory, AllTest) {

  cv::Mat original_image =
      cv::imread(atlas::kWorkspaceRoot + "src/lib_vision/test/img/FeatureFactoryTestImage.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat binary_image;
  cv::cvtColor(original_image, binary_image, CV_RGB2GRAY);
  cv::Mat debug = cv::Mat::zeros(original_image.rows, original_image.cols, CV_8UC3);
  ContourList list(binary_image, ContourList::OUTER);

  ASSERT_EQ(list.GetSize(), 4);

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
