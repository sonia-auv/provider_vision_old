/**
 * \file	feature_factory.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "lib_vision/algorithm/object_feature_factory.h"

//=============================================================================
// 	CONSTRUCTOR CODE SECTION
//-----------------------------------------------------------------------------
//
ObjectFeatureFactory::ObjectFeatureFactory(unsigned int memorySize)
    :_frame_memory(memorySize) {
  feature_fct_map.emplace(ObjectFeatureData::Feature::RATIO,
                          ObjectFeatureFactory::RatioFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::CONVEXITY,
                          ObjectFeatureFactory::ConvexityFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::PERCENT_FILLED,
                          ObjectFeatureFactory::PercentFilledFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::CIRCULARITY,
                          ObjectFeatureFactory::CircularityFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::PRESENCE_CONSISTENCY,
                          ObjectFeatureFactory::PresenceConsistencyFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::HUE_MEAN,
                          ObjectFeatureFactory::HueMeanFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::SAT_MEAN,
                          ObjectFeatureFactory::SatMeanFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::INTENSITY_MEAN,
                          ObjectFeatureFactory::IntensityMeanFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::RED_MEAN,
                          ObjectFeatureFactory::RedMeanFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::GREEN_MEAN,
                          ObjectFeatureFactory::GreenMeanFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::BLUE_MEAN,
                          ObjectFeatureFactory::BlueMeanFeature);
  feature_fct_map.emplace(ObjectFeatureData::Feature::GRAY_MEAN,
                          ObjectFeatureFactory::GrayMeanFeature);
}

//=============================================================================
// 	METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
void ObjectFeatureFactory::PercentFilledFeature(ObjectFullData::Ptr object) {
  if (object) {
    float percentFilled = 0.0f;
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
    object->SetPercentFilled(percentFilled);
  }
}

//-----------------------------------------------------------------------------
//
float ObjectFeatureFactory::CalculatePlaneMean(ObjectFullData::Ptr object,
                                               int plane) {
  float mean = 0.0f;
  if (object.get() != nullptr) {
    cv::Mat binaryImage(object->GetImageSize(), CV_8UC3, cv::Scalar::all(0));
    contourList_t contours;
    contours.push_back(object->GetContourCopy());
    cv::drawContours(binaryImage, contours, -1, CV_RGB(255, 255, 255), -1);
    cv::cvtColor(binaryImage, binaryImage, CV_BGR2GRAY);
    cv::Mat colorbinaryImage;

    cv::bitwise_and(object->GetPlanes(plane),
                    cv::Scalar::all(255), colorbinaryImage, binaryImage);
    long unsigned int accumulator = 0, nbPixel = 0;

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
    if (nbPixel != 0) {
      mean = (float(accumulator) / float(nbPixel * 255));
    }
  }
  return mean;
}