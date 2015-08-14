/**
 * \file	features.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_FEATURES_H_
#define VISION_FILTER_FEATURES_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>

#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/moments.h>
#include <lib_vision/algorithm/rot_rect.h>

/*
 * Description level:
 * All description stores the center, the angle, the corners,
 * the width and the height of the contour.
 *
 * Basic calculates the area
 *
 * Standard is basic description plus the ratio
 *
 * High Level is standard + the convexity defectuosity
 *
 */
class Features : public RotRect {
 public:
  // Enums
  enum DescLevel { DESC_BASIC = 0, DESC_STANDARD, DESC_HIGH_LEVEL };
  enum SingleFeature {
    AREA,
    RATIO,
    MOMENTS,
    PERCENT_FILLED,
    CONVEXITY_VALUE,
    MEANS,
    CIRCULARITY_INDEX,
    ARC_LENGTH
  };

  // Construction Destruction
  Features(const contour_t &contour, const cv::Mat &originalImage,
           const int descLevel);

  Features(const Features &a);

  Features(const contour_t &contour, const cv::Mat &originalImage,
           const std::vector<Features::SingleFeature> singleFeatVec);

  Features();

  ~Features();

  // Operator Overload
  void swap(Features &a);

  Features &operator=(Features feat);

  // Debug / filter usage function
  // Put in the Target object the info of Features
  void drawFeature(cv::Mat &image, const cv::Scalar contourColor,
                   const cv::Scalar centerColor, int thickness = 2);

  // Easy feature upgrade function
  void UpgradeToStandard();

  void UpgradeToHighLevel();

  // Private accessor get/set
  bool IsEmpty();

  // Those do not need setter since done at construction
  //
  const cv::Mat getOriginalImage() const;

  const contour_t getFeatureContour() const;

  const cv::Point getOffsetCenter() const;

  // From the RotRect heritance
  const cv::Point getCenter() const;

  const float getWidth() const;

  const float getHeight() const;

  const float getAngle() const;

  // Feature that need extra op. to be calculated
  const float getArea() const;

  const float getRatio() const;

  const float getConvexityValue() const;

  const float getPercentFilled() const;

  const float getCircularityValue() const;

  const float getArcLength() const;

  const cv::Scalar getBGRMean() const;

  const cv::Scalar getHSIMean() const;

  const Moments getMoments() const;

  void calcRatio();

  void calcArea();

  void calcConvexityValue();

  void calcPercentFilled();

  void calcGetCircularityValue();

  void calcArcLength();

  void calcMeans(bool middle);

  void calcMoments();

 private:
  bool empty;
  int descLevel;
  // Memory for other feature calculation
  contour_t contour;
  cv::Mat originalImage;

  // Features include in the rotated rect calculation
  cv::Point offsetedCenter;

  // Basic features
  float area;

  // Standard featrues
  float ratio;

  // High Level features
  float convexityValue;
  float percentFilled;

  // Other feature
  cv::Scalar bgrMeans;
  cv::Scalar hsvMeans;
  Moments moments;
  float circularityValue;
  float arcLength;

  void InitValues();

  // Function to fill the different values
  void FillBasicDesc();

  void FillStandardDesc();

  void FillHighLevelDesc();

  void calculateEnumCorrespondingValue(Features::SingleFeature feat);
};

//**************************************************************
//			GETTERS
//**************************************************************
inline const cv::Mat Features::getOriginalImage() const {
  return originalImage;
}

//==============================================================
//
inline const contour_t Features::getFeatureContour() const { return contour; }

//==============================================================
//
inline const cv::Point Features::getOffsetCenter() const {
  return offsetedCenter;
}

//==============================================================
//
inline const cv::Point Features::getCenter() const { return center; }

//==============================================================
//
inline const float Features::getWidth() const { return size.width; }

//==============================================================
//
inline const float Features::getHeight() const { return size.height; }

//==============================================================
//
inline const float Features::getAngle() const { return angle; }

//==============================================================
//
inline const float Features::getArea() const { return area; }

//==============================================================
//
inline const float Features::getRatio() const { return ratio; }

//==============================================================
//
inline const float Features::getConvexityValue() const {
  return convexityValue;
}

//==============================================================
//
inline const float Features::getPercentFilled() const { return percentFilled; }

//==============================================================
//
inline const float Features::getCircularityValue() const {
  return circularityValue;
}

//==============================================================
//
inline const float Features::getArcLength() const { return arcLength; }

//==============================================================
//
inline const cv::Scalar Features::getBGRMean() const { return bgrMeans; }

//==============================================================
//
inline const cv::Scalar Features::getHSIMean() const { return hsvMeans; }

//==============================================================
//
inline const Moments Features::getMoments() const { return moments; }

// Sorting function for std::sort.
// Enable sorting in bigger first.
// a < b = asscending
// a > b = descending
bool areaSort(Features a, Features b);

bool widthSort(Features a, Features b);

bool heightSort(Features a, Features b);

bool angleSort(Features a, Features b);

bool angleAbsSort(Features a, Features b);

bool ratioSort(Features a, Features b);

// Sort the vector in difference from 0.5 of ratio (width = half height)
// The smallest difference (the nearest 0.5) is first, the farthest
// ratio from 0.5 is the last.
bool ratioBinSort(Features a, Features b);

bool convexitySort(Features a, Features b);

bool percentFilledSort(Features a, Features b);

#endif
