/**
 * \file	ai_trainer.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/ai_trainer.h>

//=============================================================================
//	CONSTANT
const std::string AITrainer::FILE_NAME = "SEQUENCE_DATA";
const std::string AITrainer::ORIGINAL_IMAGE_NODE_NAME = "ORIGINAL_IMAGE";
const std::string AITrainer::BINARY_IMAGE_NODE_NAME = "BINARY_IMAGE";
const std::string AITrainer::CENTER_NODE_NAME = "CENTER";
const std::string AITrainer::CONTOUR_NODE_NAME = "CONTOUR";
const std::string AITrainer::NB_ELEMENT_NODE_NAME = "NB_ELEMENT";
const std::string AITrainer::FRAME_NODE_NAME = "FRAME_NODE";
const std::string AITrainer::FEATURES_MAT_NODE_NAME = "FEATURES";

//=============================================================================
//	METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
void AITrainer::OutputFrameData(const std::string &outputFolderPath,
                                ObjectFullData::FullObjectPtrVec objects,
                                const cv::Mat &originalImage,
                                const cv::Mat &binaryImage,
                                unsigned int frameIndex) {
  CLString fileName;
  fileName.Format("%s/%s_%.5d.xml", outputFolderPath.c_str(), FILE_NAME.c_str(),
                  frameIndex);

  cv::FileStorage fs;
  fs.open(fileName.c_str(), cv::FileStorage::WRITE | cv::FileStorage::APPEND);
  if (!fs.isOpened()) {
    printf("Error opening file storage\n");
    return;
  }

  fs << FRAME_NODE_NAME.c_str() << int(frameIndex);
  // Save original image
  fileName.Format("%s/%s_%.5d.jpeg", outputFolderPath.c_str(),
                  ORIGINAL_IMAGE_NODE_NAME.c_str(), frameIndex);
  cv::imwrite(fileName.c_str(), originalImage);
  fs << ORIGINAL_IMAGE_NODE_NAME.c_str() << fileName.c_str();

  // Save binary image
  fileName.Format("%s/%s_%.5d.jpeg", outputFolderPath.c_str(),
                  BINARY_IMAGE_NODE_NAME.c_str(), frameIndex);
  cv::imwrite(fileName.c_str(), binaryImage);
  fs << BINARY_IMAGE_NODE_NAME.c_str() << fileName.c_str();

  fs << NB_ELEMENT_NODE_NAME.c_str() << int(objects.size());
  // Save the objects of the frame
  for (int i = 0; i < objects.size(); i++) {
    if (objects[i].IsNotNull()) {
      CLString nodeName;
      nodeName.Format("%s_%.5d", CENTER_NODE_NAME.c_str(), i);
      cv::Point2f center = objects[i]->GetCenter();
      fs << nodeName.c_str() << center;
      nodeName.Format("%s_%.5d", CONTOUR_NODE_NAME.c_str(), i);
      fs << nodeName.c_str() << objects[i]->GetContourCopy();
      nodeName.Format("%s_%.5d", FEATURES_MAT_NODE_NAME.c_str(), i);
      fs << nodeName.c_str() << objects[i]->OutputVectorAsMat();
    }
  }
  fs.release();
}

//-----------------------------------------------------------------------------
//
void AITrainer::ParseDataFolderList(
    const std::vector<std::string> outputFolderPathList,
    const std::string &outputFolderPath) {
  ParsingData parsingData;
  for (int i = 0; i < outputFolderPathList.size(); i++) {
    ParseDataFolder(outputFolderPathList[i], parsingData);
  }
  CLString fileName;
  std::ofstream file;
  fileName.Format("%s/positiveSet.cvs", outputFolderPath.c_str());
  file.open(fileName.c_str(), std::ios::out | std::ios::app);
  for (int i = 0; i < parsingData._positive_data.size(); i++) {
    file << parsingData._positive_data[i].GenerateCVine() << "\n";
  }
  file.close();

  fileName.Format("%s/negativeSet.cvs", outputFolderPath.c_str());
  file.open(fileName.c_str(), std::ios::out | std::ios::app);
  for (int i = 0; i < parsingData._negative_data.size(); i++) {
    file << parsingData._negative_data[i].GenerateCVine() << "\n";
  }
  file.close();
}

//-----------------------------------------------------------------------------
//
void AITrainer::OutputTrainingDataCVS(const std::string &positivePath,
                                      const std::string &negativePath) {}

//-----------------------------------------------------------------------------
//
void AITrainer::CallBackFunc(int event, int x, int y, int flags,
                             void *userdata) {
  cv::Point2f click(x, y);

  ParsingData *trainer = static_cast<ParsingData *>(userdata);
  if (trainer == NULL || trainer->_data_to_be_parsed.size() < 1) {
    return;
  }

  if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN) {
    RegisteredData nearestData;
    float minDist = 1000000.0f;
    int dataIndex = -1;

    for (int i = 0; i < trainer->_data_to_be_parsed.size(); i++) {
      float distance =
          eucledianPointDistance(click, trainer->_data_to_be_parsed[i].center);

      if (distance < minDist) {
        nearestData = trainer->_data_to_be_parsed[i];
        minDist = distance;
        dataIndex = i;
      }
    }

    if (dataIndex == -1) return;

    FeatureVec vec;
    vec.CreateFromMat(trainer->_data_to_be_parsed[dataIndex].features);
    if (event == cv::EVENT_LBUTTONDOWN) {
      trainer->_positive_data.push_back(vec);
    } else if (event == cv::EVENT_RBUTTONDOWN) {
      trainer->_negative_data.push_back(vec);
    }
  }
}

//-----------------------------------------------------------------------------
//
void AITrainer::ParseDataFolder(const std::string &outputFolderPath,
                                ParsingData &parsingData) {
  cv::FileStorage fs;
  unsigned int frameIndex = 0;
  CLString fileName;
  fileName.Format("%s/%s_%.5d.xml", outputFolderPath.c_str(),
                  AITrainer::FILE_NAME.c_str(), frameIndex);

  while (fs.open(fileName.c_str(), cv::FileStorage::READ)) {
    ParseFrame(fs, parsingData);
    fs.release();
    frameIndex++;
    fileName.Format("%s/%s_%.5d.xml", outputFolderPath.c_str(),
                    AITrainer::FILE_NAME.c_str(), frameIndex);
  }
}

//-----------------------------------------------------------------------------
//
void AITrainer::ParseFrame(const cv::FileStorage &openedFs,
                           ParsingData &parsingData) {
  int frameIndex = (int)openedFs[AITrainer::FRAME_NODE_NAME.c_str()];

  std::string tmp =
      (std::string)openedFs[AITrainer::ORIGINAL_IMAGE_NODE_NAME.c_str()];
  cv::Mat originalImage = cv::imread(tmp);

  tmp = (std::string)openedFs[AITrainer::BINARY_IMAGE_NODE_NAME.c_str()];
  cv::Mat binaryImage = cv::imread(tmp);

  int nbElement = (int)openedFs[AITrainer::NB_ELEMENT_NODE_NAME.c_str()];

  parsingData._data_to_be_parsed.clear();
  cv::Mat queryImageOriginal = originalImage.clone();
  cv::Mat queryImageBinary = binaryImage.clone();
  if (queryImageBinary.channels() == 1)
    cv::cvtColor(queryImageBinary, queryImageBinary, CV_GRAY2BGR);

  contourList_t contourToDraw;
  for (int i = 0; i < nbElement; i++) {
    RegisteredData tmp;
    CLString nodeName;

    nodeName.Format("%s_%.5d", AITrainer::CENTER_NODE_NAME.c_str(), i);
    openedFs[nodeName.c_str()] >> tmp.center;

    nodeName.Format("%s_%.5d", AITrainer::CONTOUR_NODE_NAME.c_str(), i);
    openedFs[nodeName.c_str()] >> tmp.contour;

    nodeName.Format("%s_%.5d", AITrainer::FEATURES_MAT_NODE_NAME.c_str(), i);
    openedFs[nodeName.c_str()] >> tmp.features;
    parsingData._data_to_be_parsed.push_back(tmp);
    cv::circle(queryImageBinary, tmp.center, 2, CV_RGB(255, 0, 0), -1);
    contourToDraw.push_back(tmp.contour);
  };

  cv::drawContours(queryImageBinary, contourToDraw, -1, CV_RGB(255, 0, 0), 3);
  //	parsingData.drawingImage = queryImageBinary;

  cv::imshow("binary", queryImageBinary);
  cv::setMouseCallback("binary", AITrainer::CallBackFunc, &parsingData);
  cv::waitKey(0);
}

//=============================================================================
//=============================================================================
//=============================================================================
//=========================== UNIT TEST AREA ==================================
//=============================================================================
//=============================================================================
//=============================================================================
#include <TCUnitTest.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/feature_factory.h>
#include <lib_vision/algorithm/object_full_data.h>

TC_DEFINE_UNIT_TEST(AITrainerUT) {
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
    objectVector.push_back(
        new ObjectFullData(originalImage, binaryImage, contours[i]));
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
  return true;
}
TC_END_UNIT_TEST(AITrainerUT);

TC_DEFINE_UNIT_TEST(AITrainerFeatureDataRecomposerUT) {
  std::vector<std::string> tmp;
  std::string folderPath("/home/jeremie/aidata/rec1");
  tmp.push_back(folderPath);
  //	folderPath = "/home/jeremie/aidata/rec4";
  //	tmp.push_back(folderPath);
  AITrainer trainer;
  trainer.ParseDataFolderList(tmp, "/home/jeremie/aidata/");

  return true;
}
TC_END_UNIT_TEST(AITrainerFeatureDataRecomposerUT);
