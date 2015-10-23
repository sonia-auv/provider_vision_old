///**
// * \file	ai_trainer.h
// * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
// * \date	1/01/2014
// * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
// * Use of this source code is governed by the MIT license that can be
// * found in the LICENSE file.
// */
//
//#ifndef VISION_FILTER_AI_TRAINER_H_
//#define VISION_FILTER_AI_TRAINER_H_
//
//#include <string>
//#include <opencv2/opencv.hpp>
//#include <iostream>
//#include <fstream>
//#include <lib_vision/algorithm/object_full_data.h>
//#include <lib_vision/algorithm/general_function.h>
//
//class AITrainer {
// public:
//  static const std::string FILE_NAME;
//  static const std::string ORIGINAL_IMAGE_NODE_NAME;
//  static const std::string BINARY_IMAGE_NODE_NAME;
//  static const std::string CENTER_NODE_NAME;
//  static const std::string CONTOUR_NODE_NAME;
//  static const std::string NB_ELEMENT_NODE_NAME;
//  static const std::string FRAME_NODE_NAME;
//  static const std::string FEATURES_MAT_NODE_NAME;
//
//  static void OutputFrameData(const std::string &outputFolderPath,
//                              ObjectFullData::FullObjectPtrVec objects,
//                              const cv::Mat &originalImage,
//                              const cv::Mat &binaryImage,
//                              unsigned int frameIndex);
//
//  // Data that is going to be read back from the node. Should be a class
//  // handling
//  // writing and reading
//  typedef struct {
//    cv::Point2f center;
//    contour_t contour;
//    cv::Mat features;
//  } RegisteredData;
//
//  typedef struct {
//    std::vector<ObjectFullData::Ptr> _positive_data, _negative_data;
//    std::vector<RegisteredData> _data_to_be_parsed;
//    //		cv::Mat drawingImage;
//  } ParsingData;
//
//  void ParseDataFolderList(const std::vector<std::string> folderPathList,
//                           const std::string &outputFolderPath);
//
//  static void OutputTrainingDataCVS(const std::string &positivePath,
//                                    const std::string &negativePath);
//
// private:
//  static void CallBackFunc(int event, int x, int y, int flags, void *userdata);
//
//  static void ParseDataFolder(const std::string &outputFolderPath,
//                              ParsingData &parsingData);
//
//  static void ParseFrame(const cv::FileStorage &openedFs,
//                         ParsingData &parsingData);
//};
//
//#endif
