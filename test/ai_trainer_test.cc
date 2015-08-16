/**
 * \file	ai_trainer_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/ai_trainer.h>
#include <lib_vision/algorithm/feature_factory.h>

TEST(AITrainer, AllTest) {
  printf("Starting unit test on AITrainer\n");

  cv::Mat originalImage = cv::imread("/home/jeremie/Videos/test_contours.jpg");
  cv::Mat binaryImage;
  cv::cvtColor(originalImage, binaryImage, CV_BGR2GRAY);
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
  FeatureFactory featFactory(3);
  featFactory.SetFeatureToCompute(feature);
  // fills buffer and save data

  featFactory.CalculateFeatureVectors(objectVector);
  AITrainer::OutputFrameData("/home/jeremie/aidata/", objectVector,
                             originalImage, binaryImage, 0);
  featFactory.CalculateFeatureVectors(objectVector);
  AITrainer::OutputFrameData("/home/jeremie/aidata/", objectVector,
                             originalImage, binaryImage, 1);
  featFactory.CalculateFeatureVectors(objectVector);
  AITrainer::OutputFrameData("/home/jeremie/aidata/", objectVector,
                             originalImage, binaryImage, 2);
  featFactory.CalculateFeatureVectors(objectVector);
  AITrainer::OutputFrameData("/home/jeremie/aidata/", objectVector,
                             originalImage, binaryImage, 3);

  printf("System all clear and good to go\n");
}

TEST(AITrainer, DataRecomposer) {
  std::vector<std::string> tmp;
  std::string folderPath("/home/jeremie/aidata/rec1");
  tmp.push_back(folderPath);
  //	folderPath = "/home/jeremie/aidata/rec4";
  //	tmp.push_back(folderPath);
  AITrainer trainer;
  trainer.ParseDataFolderList(tmp, "/home/jeremie/aidata/");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}