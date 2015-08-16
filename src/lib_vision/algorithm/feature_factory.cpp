/**
 * \file	feature_factory.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/feature_factory.h>

//=============================================================================
// 	CONSTRUCTOR CODE SECTION
//-----------------------------------------------------------------------------
//
FeatureFactory::FeatureFactory(unsigned int memorySize)
    : _calculate_area_rank(false),
      _calculate_length_rank(false),
      _frame_memory(memorySize) {}

//=============================================================================
// 	METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
void FeatureFactory::CalculateFeatureVectors(
    ObjectFullData::FullObjectPtrVec objects) {
  std::vector<FeatureVec> objectsFeatures;
  if (_calculate_area_rank) {
    ObjectRanker::RankByArea(objects);
  }
  if (_calculate_length_rank) {
    ObjectRanker::RankByLength(objects);
  }

  for (int j = 0, sizeObjects = objects.size(); j < sizeObjects; j++) {
    if (objects[j].get() != nullptr) {
      for (int i = 0, sizeFeature = _feature_calculator_vector.size();
           i < sizeFeature; i++) {
        objects[j]->AddFeature(
            (this->*(_feature_calculator_vector[i]))(objects[j]));
      }
    }
    _frame_memory.AddFrameObjects(objects);
  }
}

//-----------------------------------------------------------------------------
//
void FeatureFactory::SetFeatureToCompute(
    std::vector<FEATURE_TYPE> featureToCompute) {
  for (int i = 0; i < featureToCompute.size(); i++) {
    switch (featureToCompute[i]) {
      case AREA_RANK:
        _calculate_area_rank = true;
        _feature_calculator_vector.push_back(&FeatureFactory::AreaFeature);
        break;
      case LENGTH_RANK:
        _calculate_length_rank = true;
        _feature_calculator_vector.push_back(&FeatureFactory::LengthFeature);
        break;
      case RATIO:
        _feature_calculator_vector.push_back(&FeatureFactory::RatioFeature);
        break;
      case CONVEXITY:
        _feature_calculator_vector.push_back(&FeatureFactory::ConvexityFeature);
        break;
      case PERCENT_FILLED:
        _feature_calculator_vector.push_back(
            &FeatureFactory::PercentFilledFeature);
        break;
      case CIRCULARITY:
        _feature_calculator_vector.push_back(
            &FeatureFactory::CircularityFeature);
        break;
      case PRESENCE_CONSISTENCY:
        _feature_calculator_vector.push_back(
            &FeatureFactory::PresenceConsistencyFeature);
        break;
      case HUE_MEAN:
        _feature_calculator_vector.push_back(&FeatureFactory::HueMeanFeature);
        break;
      default:
        std::printf("FEatureFactory: Bad feature type\n");
        break;
    }
  }
}

//-----------------------------------------------------------------------------
//
void FeatureFactory::SetAllFeatureToCompute() {
  _calculate_area_rank = true;
  _calculate_length_rank = true;
  // For cvs:
  // area, length, ratio, convexity, percentfilled, circularity, presence,
  // hueMean
  _feature_calculator_vector.push_back(&FeatureFactory::AreaFeature);
  _feature_calculator_vector.push_back(&FeatureFactory::LengthFeature);
  _feature_calculator_vector.push_back(&FeatureFactory::RatioFeature);
  _feature_calculator_vector.push_back(&FeatureFactory::ConvexityFeature);
  _feature_calculator_vector.push_back(&FeatureFactory::PercentFilledFeature);
  _feature_calculator_vector.push_back(&FeatureFactory::CircularityFeature);
  _feature_calculator_vector.push_back(
      &FeatureFactory::PresenceConsistencyFeature);
  _feature_calculator_vector.push_back(&FeatureFactory::HueMeanFeature);
}

//-----------------------------------------------------------------------------
//
float FeatureFactory::PercentFilledFeature(
    std::shared_ptr<ObjectFullData> object) {
  float percentFilled = 0.0f;
  if (object.get() != nullptr) {
    cv::Size imageSize = object->GetImageSize();
    cv::Mat drawImage(imageSize, CV_8UC3, cv::Scalar::all(0));
    contourList_t contours;
    cv::Point2f pts[4];
    RotRect rrect = object->GetRotatedRect();
    rrect.points(pts);
    contour_t contour(4);
    for (int i = 0; i < 4; i++) {
      contour[i].x = int(pts[i].x);
      contour[i].y = int(pts[i].y);
    }
    contours.push_back(contour);
    contours.push_back(object->GetContourCopy());

    // Draw the biggest one (contour by corners)
    // Then draw the contour in black over the biggest one.
    cv::drawContours(drawImage, contours, 0, CV_RGB(255, 255, 255), -1);
    cv::drawContours(drawImage, contours, 1, CV_RGB(0, 0, 0), -1);

    cv::cvtColor(drawImage, drawImage, CV_BGR2GRAY);
    float notCovered = cv::countNonZero(drawImage);
    // safety, should not happen
    float rrectArea = rrect.size.area();
    if (rrectArea != 0) percentFilled = 1.0f - (notCovered / rrectArea);
  }
  return percentFilled;
}

//-----------------------------------------------------------------------------
//
float FeatureFactory::HueMeanFeature(std::shared_ptr<ObjectFullData> object) {
  float hueMean = 0.0f;
  if (object.get() != nullptr) {
    cv::Mat binaryImage(object->GetImageSize(), CV_8UC3, cv::Scalar::all(0));
    contourList_t contours;
    contours.push_back(object->GetContourCopy());
    cv::drawContours(binaryImage, contours, -1, CV_RGB(255, 255, 255), -1);
    cv::cvtColor(binaryImage, binaryImage, CV_BGR2GRAY);
    cv::Mat colorbinaryImage;

    cv::bitwise_and(object->GetPlanes(ObjectBasicData::HUE_PLANE),
                    cv::Scalar::all(255), colorbinaryImage, binaryImage);
    long unsigned int accumulator = 0, nbPixel = 0;
    ;
    int rows = colorbinaryImage.rows, cols = colorbinaryImage.cols;
    for (int y = 0; y < rows; y++) {
      uchar *ptr = colorbinaryImage.ptr<uchar>(y);
      for (int x = 0; x < cols; x++) {
        if (ptr[x] != 0) {
          accumulator += ptr[x];
          nbPixel++;
        }
      }
    }
    if (nbPixel != 0) hueMean = (float(accumulator) / float(nbPixel * 255));
  }
  return hueMean;
}
