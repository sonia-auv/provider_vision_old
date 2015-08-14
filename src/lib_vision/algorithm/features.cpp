/**
 * \file	features.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/features.h>

Features::Features(const contour_t &contour, const cv::Mat &originalImage,
                   const int descLevel)
    : RotRect(contour) {
  InitValues();

  this->empty = false;

  this->descLevel = descLevel;
  this->offsetedCenter = this->center;
  setCameraOffset(&this->offsetedCenter, originalImage.rows,
                  originalImage.cols);
  this->originalImage = originalImage;
  this->contour = contour;

  switch (descLevel) {
    case DESC_HIGH_LEVEL:
      FillHighLevelDesc();
    case DESC_STANDARD:
      FillStandardDesc();
    case DESC_BASIC:
      FillBasicDesc();
      break;
  }
}

//==============================================================
//
Features::Features(const Features &a)
    : RotRect(a),
      empty(a.empty),
      descLevel(a.descLevel),
      contour(a.contour),
      originalImage(a.originalImage),
      offsetedCenter(a.offsetedCenter),
      area(a.area),
      ratio(a.ratio),
      convexityValue(a.convexityValue),
      percentFilled(a.percentFilled),
      bgrMeans(a.bgrMeans),
      hsvMeans(a.hsvMeans),
      moments(a.moments),
      circularityValue(a.circularityValue),
      arcLength(a.arcLength) {}

//==============================================================
//
Features::Features(const contour_t &contour, const cv::Mat &originalImage,
                   const std::vector<Features::SingleFeature> singleFeatVec)
    : RotRect(contour) {
  InitValues();

  this->empty = false;

  this->offsetedCenter = this->center;
  setCameraOffset(&this->offsetedCenter, originalImage.rows,
                  originalImage.cols);
  this->originalImage = originalImage;
  this->contour = contour;

  for (int i = 0; i < static_cast<int>(singleFeatVec.size()); i++) {
    calculateEnumCorrespondingValue(singleFeatVec[i]);
  }
}

//==============================================================
//
Features::Features() : RotRect() { InitValues(); }

//==============================================================
//
Features::~Features() {}

//==============================================================
//
void Features::swap(Features &a) {
  std::swap(a.empty, this->empty);
  std::swap(a.descLevel, this->descLevel);
  std::swap(a.offsetedCenter, this->offsetedCenter);
  std::swap(a.area, this->area);
  std::swap(a.ratio, this->ratio);
  std::swap(a.convexityValue, this->convexityValue);
  std::swap(a.percentFilled, this->percentFilled);
  std::swap(a.moments, this->moments);
  std::swap(a.bgrMeans, this->bgrMeans);
  std::swap(a.hsvMeans, this->hsvMeans);
  std::swap(a.circularityValue, this->circularityValue);
  std::swap(a.arcLength, this->arcLength);
}

//==============================================================
//
Features &Features::operator=(Features feat) {
  RotRect::operator=(feat);
  swap(feat);
  return *this;
}

//==============================================================
//
void Features::drawFeature(cv::Mat &image, cv::Scalar contourColor,
                           cv::Scalar centerColor, int thickness) {
  if (image.channels() != 3) return;
  this->drawRect(image, contourColor, thickness);
  cv::circle(image, this->center, thickness, centerColor, thickness);
}

//==============================================================
//
void Features::UpgradeToStandard() {
  FillStandardDesc();
  this->descLevel = DESC_STANDARD;
}

//==============================================================
//
void Features::UpgradeToHighLevel() {
  if (this->descLevel == DESC_BASIC) FillStandardDesc();
  FillHighLevelDesc();
  this->descLevel = DESC_HIGH_LEVEL;
}

//==============================================================
//
void Features::FillBasicDesc() { this->calcArea(); }

//==============================================================
//
void Features::FillStandardDesc() { this->calcRatio(); }

//==============================================================
//
void Features::FillHighLevelDesc() {
  this->calcConvexityValue();
  this->calcPercentFilled();
}

//==============================================================
//
void Features::calculateEnumCorrespondingValue(Features::SingleFeature feat) {
  switch (feat) {
    case (AREA):
      calcArea();
      break;
    case (RATIO):
      calcRatio();
      break;
    case (MOMENTS):
      calcMoments();
      break;
    case (PERCENT_FILLED):
      calcPercentFilled();
      break;
    case (CONVEXITY_VALUE):
      calcConvexityValue();
      break;
    case (MEANS):
      calcMeans(true);
      break;
    case (CIRCULARITY_INDEX):
      calcGetCircularityValue();
      break;
    case (ARC_LENGTH):
      calcArcLength();
      break;
  }
}

//==============================================================
//
bool Features::IsEmpty() { return empty; }

//==============================================================
//
void Features::InitValues() {
  this->empty = true;
  this->descLevel = -1;
  this->offsetedCenter = cv::Point(-1, -1);

  // Basic
  this->area = -1.0f;
  // Standard
  this->ratio = -1.0f;
  // High Level
  this->convexityValue = -1.0;
  this->percentFilled = -1.0f;

  this->moments = Moments();
  this->bgrMeans = 0;
  this->hsvMeans = 0;
  this->circularityValue = 0.0f;
  this->arcLength = 0.0f;
}

//**************************************************************
//			Feature Calculation
//**************************************************************
void Features::calcArea() {
  this->area = cv::contourArea(this->contour, false);
}

//==============================================================
//
void Features::calcRatio() {
  this->ratio = calculateRatio(this->size.width, this->size.height);
}

//==============================================================
//
void Features::calcConvexityValue() {
  this->convexityValue = calculateConvexityRatio(this->contour);
}

//==============================================================
//
void Features::calcPercentFilled() {
  this->percentFilled = calculatePourcentFilled(this->originalImage, *this);
}

//==============================================================
//
void Features::calcGetCircularityValue() {
  this->circularityValue = calculateCircleIndex(this->contour);
}

//==============================================================
//
void Features::calcArcLength() {
  this->arcLength = cv::arcLength(this->contour, true);
}

//==============================================================
//
void Features::calcMeans(bool middle) {
  if (this->originalImage.channels() > 1) {
    cv::Mat hsi;
    cv::cvtColor(this->originalImage, hsi, CV_BGR2HSV);
    this->bgrMeans = calculateMeans(this->contour, this->originalImage, middle);
    this->hsvMeans = calculateMeans(this->contour, hsi, middle);
  } else {
    this->bgrMeans = calculateMeans(this->contour, this->originalImage, middle);
    this->hsvMeans = this->bgrMeans;
  }
}

//==============================================================
//
void Features::calcMoments() {
  cv::Mat momImage = extractImageFromRect(contour, originalImage);
  this->moments = Moments(momImage, true);
}

//**************************************************************
//			STD::SORT FUNCTION
//**************************************************************
bool areaSort(Features a, Features b) { return a.getArea() > b.getArea(); }

//==============================================================
//
bool widthSort(Features a, Features b) { return a.size.height > b.size.width; }

//==============================================================
//
bool heightSort(Features a, Features b) {
  return a.size.height > b.size.height;
}

//==============================================================
//
bool angleSort(Features a, Features b) { return a.angle > b.angle; }

//==============================================================
//
bool angleAbsSort(Features a, Features b) {
  return abs(a.angle) > abs(b.angle);
}

//==============================================================
//
bool ratioSort(Features a, Features b) { return a.getRatio() > b.getRatio(); }

//==============================================================
//
bool ratioBinSort(Features a, Features b) {
  return abs((0.5 - a.getRatio()) * 100) < abs((0.5 - b.getRatio()) * 100);
}

//==============================================================
//
bool convexitySort(Features a, Features b) {
  return a.getConvexityValue() > b.getConvexityValue();
}

//==============================================================
//
bool percentFilledSort(Features a, Features b) {
  return a.getPercentFilled() > b.getPercentFilled();
}
