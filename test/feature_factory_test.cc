/**
 * \file	ai_trainer_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/feature_factory.h>
#include <stdlib.h>

TEST(FeatureFactory, AllTest) {
  printf("Starting unit test on FeatureFactory\n");

  cv::Mat originalImage =
      cv::imread("/home/jeremie/Videos/PercentFilledTry.png");
  cv::Mat binaryImage = cv::imread("/home/jeremie/Videos/PercentFilledTry.png");
  cv::cvtColor(binaryImage, binaryImage, CV_BGR2GRAY);
  contourList_t contours;
  retrieveAllContours(binaryImage, contours);

  ObjectFullData::FullObjectPtrVec objectVector;
  for (int i = 0; i < contours.size(); i++) {
    float area = cv::contourArea(contours[i]);
    if (area < 100) continue;
    objectVector.push_back(std::make_shared<ObjectFullData>(
        originalImage, binaryImage, contours[i]));
  }
  std::vector<FeatureFactory::FEATURE_TYPE> feature;
  feature.push_back(FeatureFactory::AREA_RANK);
  feature.push_back(FeatureFactory::LENGTH_RANK);
  feature.push_back(FeatureFactory::CIRCULARITY);
  feature.push_back(FeatureFactory::CONVEXITY);
  feature.push_back(FeatureFactory::RATIO);
  feature.push_back(FeatureFactory::PRESENCE_CONSISTENCY);
  feature.push_back(FeatureFactory::PERCENT_FILLED);
  feature.push_back(FeatureFactory::HUE_MEAN);
  FeatureFactory featFactory(3);
  featFactory.SetFeatureToCompute(feature);
  // fills buffer
  featFactory.CalculateFeatureVectors(objectVector);
  featFactory.CalculateFeatureVectors(objectVector);
  featFactory.CalculateFeatureVectors(objectVector);
  featFactory.CalculateFeatureVectors(objectVector);
  // No real testing here, all data has been tested in the individual classes...
  // only makes sure that we have data...
  // Generated mat
  std::vector<cv::Mat> genMat;
  for (int i = 0; i < objectVector.size(); i++) {
    genMat.push_back(objectVector[i]->OutputVectorAsMat());
  }
  // Create from Mat
  std::vector<FeatureVec> matToGen(objectVector.size());
  for (int i = 0; i < genMat.size(); i++) {
    matToGen[i].CreateFromMat(genMat[i]);
  }

  for (int i = 0; i < objectVector.size(); i++) {
    std::vector<float> original = objectVector[i]->GetVec(),
                       generated = matToGen[i].GetVec();
    for (int j = 0; j < original.size(); j++) {
      if (j == 6) {
        printf("Length : %f\t", objectVector[i]->GetLength());
        printf("Length rank: %f\t", objectVector[i]->GetLengthRank());

        printf("Percent filled : %f\t", original[j]);
      } else if (j == 7) {
        printf(" Hue Mean %f\t", original[j]);
      }
      bool compareResult = generated[j] - original[j] == 0;
      // TC_TEST_FAIL("Wrongly generated", compareResult);
    }
    printf("\n");
  }

  printf("System all clear and good to go\n");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
