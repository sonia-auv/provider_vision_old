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
  std::string fileName =
      outputFolderPath + FILE_NAME + std::to_string(frameIndex);

  cv::FileStorage fs;
  fs.open(fileName.c_str(), cv::FileStorage::WRITE | cv::FileStorage::APPEND);
  if (!fs.isOpened()) {
    printf("Error opening file storage\n");
    return;
  }

  fs << FRAME_NODE_NAME.c_str() << int(frameIndex);
  // Save original image
  fileName =
      outputFolderPath + ORIGINAL_IMAGE_NODE_NAME + std::to_string(frameIndex);
  cv::imwrite(fileName.c_str(), originalImage);
  fs << ORIGINAL_IMAGE_NODE_NAME.c_str() << fileName.c_str();

  // Save binary image
  fileName =
      outputFolderPath + BINARY_IMAGE_NODE_NAME + std::to_string(frameIndex);
  cv::imwrite(fileName.c_str(), binaryImage);
  fs << BINARY_IMAGE_NODE_NAME.c_str() << fileName.c_str();

  fs << NB_ELEMENT_NODE_NAME.c_str() << int(objects.size());
  // Save the objects of the frame
  for (int i = 0; i < objects.size(); i++) {
    if (objects[i].get() != nullptr) {
      std::string nodeName = CENTER_NODE_NAME + std::to_string(i);
      cv::Point2f center = objects[i]->GetCenter();
      fs << nodeName.c_str() << center;
      nodeName = CONTOUR_NODE_NAME + std::to_string(i);
      fs << nodeName.c_str() << objects[i]->GetContourCopy();
      nodeName = FEATURES_MAT_NODE_NAME + std::to_string(i);
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
  std::string fileName = outputFolderPath + "/positiveSet.cvs";
  std::ofstream file;
  file.open(fileName.c_str(), std::ios::out | std::ios::app);
  for (int i = 0; i < parsingData._positive_data.size(); i++) {
    file << parsingData._positive_data[i].GenerateCVine() << "\n";
  }
  file.close();

  fileName = outputFolderPath + "/negativeSet.cvs";
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
  std::string fileName =
      outputFolderPath + FILE_NAME + std::to_string(frameIndex);

  while (fs.open(fileName.c_str(), cv::FileStorage::READ)) {
    ParseFrame(fs, parsingData);
    fs.release();
    frameIndex++;
    fileName = outputFolderPath + FILE_NAME + std::to_string(frameIndex);
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
    std::string nodeName = CENTER_NODE_NAME + "_" + std::to_string(i);
    openedFs[nodeName.c_str()] >> tmp.center;

    nodeName = CONTOUR_NODE_NAME + "_" + std::to_string(i);
    openedFs[nodeName.c_str()] >> tmp.contour;

    nodeName = FEATURES_MAT_NODE_NAME + "_" + std::to_string(i);
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
