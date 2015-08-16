/**
 * \file	object_basic_data.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_BASIC_OBJECT_DATA_H_
#define VISION_FILTER_BASIC_OBJECT_DATA_H_

#include <lib_vision/algorithm/rot_rect.h>
#include <lib_vision/algorithm/type_and_const.h>
#include <assert.h>

class ObjectBasicData {
 public:
  static const int BLUE_PLANE = 0;
  static const int GREEN_PLANE = 1;
  static const int RED_PLANE = 2;
  static const int HUE_PLANE = 3;
  static const int SATURATION_PLANE = 4;
  static const int INTENSITY_PLANE = 5;
  static const int GRAY_PLANE = 6;
  static const int NB_OF_PLANE = 7;

  enum OBJECT_DATA {
    AREA,
    RATIO,
    CONVEX_HULL,
    CIRCUMFERENCE,
    ROTATED_RECT,
    UP_RIGHT_RECT,
    MOMENTS,
    PLANES,
    AREA_RANK,
    LENGTH_RANK,
    DISTANCE_FROM_CENTER
  };

  ObjectBasicData(const cv::Mat &originalImage, const cv::Mat &binaryImage,
                  const contour_t &contour);

  virtual ~ObjectBasicData() {}

  void SetPlaneInRange(int &planeID);

  // Voting system
  void IncrementVote();
  int GetVoteCount();
  void ResetVote();

  // All getters calculate their element if they are not already calculated.
  // If they are, simply return them.
  float GetArea();

  float GetLength();

  float GetRatio();

  float GetConvexHullArea();

  float GetCircumference();

  float GetDistanceFromCenter();

  const RotRect &GetRotatedRect();

  const cv::Point2f &GetCenter();

  const cv::Rect &GetUprightRect();

  const cv::Moments &GetMoments(bool useBinary);

  // Images are already reference in opencv...
  const cv::Mat GetPlanes(int planesID);

  const cv::Mat GetBinaryImageAtUprightRect();

  contour_t GetContourCopy();

  const cv::Size GetImageSize();

  const cv::Mat GetBinaryImage();

  const cv::Mat GetOriginalImage();

 private:
  std::map<OBJECT_DATA, bool> _is_calculated_map;

  int _vote_count;

  float _area, _convex_hull_area, _circumference, _distance_from_center;

  // Correspond to the centile rank
  float _areaRanking, _lengthRanking;
  RotRect _rect;
  cv::Rect _up_right_rect;
  cv::Moments _cv_moments;
  std::vector<cv::Mat> _planes;

  cv::Mat _original_image, _binary_image;
  contour_t _contour;

  friend class ObjectBasicDataUT;
};

//-----------------------------------------------------------------------------
//
inline void ObjectBasicData::IncrementVote() { _vote_count++; }

//-----------------------------------------------------------------------------
//
inline int ObjectBasicData::GetVoteCount() { return _vote_count; }

//-----------------------------------------------------------------------------
//
inline void ObjectBasicData::ResetVote() { _vote_count = 0; }

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetArea() {
  if (!_is_calculated_map[AREA]) {
    _area = cv::contourArea(_contour, false);
    _is_calculated_map[AREA] = true;
  }
  return _area;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetLength() {
  if (!_is_calculated_map[ROTATED_RECT]) {
    _rect = RotRect(_contour);
    _is_calculated_map[ROTATED_RECT] = true;
  }
  return _rect.size.height;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetRatio() {
  if (!_is_calculated_map[ROTATED_RECT]) {
    _rect = RotRect(_contour);
    _is_calculated_map[ROTATED_RECT] = true;
  }
  if (_rect.size.height != 0) return _rect.size.width / _rect.size.height;
  return 0.0f;
}

//-----------------------------------------------------------------------------
//
inline void ObjectBasicData::SetPlaneInRange(int &planeID) {
  // Clamping the planeID in [0; NB_OF_PLANE - 1]
  planeID =
      planeID < 0 ? 0 : (planeID > NB_OF_PLANE - 1 ? NB_OF_PLANE - 1 : planeID);
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetConvexHullArea() {
  if (!_is_calculated_map[CONVEX_HULL]) {
    contour_t convexHull;
    cv::convexHull(_contour, convexHull, false, true);
    _convex_hull_area = cv::contourArea(convexHull, false);
    _is_calculated_map[CONVEX_HULL] = true;
  }
  return _convex_hull_area;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetCircumference() {
  if (!_is_calculated_map[CIRCUMFERENCE]) {
    _circumference = cv::arcLength(_contour, true);
    _is_calculated_map[CIRCUMFERENCE] = true;
  }
  return _circumference;
}

//-----------------------------------------------------------------------------
//
inline const RotRect &ObjectBasicData::GetRotatedRect() {
  if (!_is_calculated_map[ROTATED_RECT]) {
    _rect = RotRect(_contour);
    _is_calculated_map[ROTATED_RECT] = true;
  }
  return _rect;
}

//-----------------------------------------------------------------------------
//
inline const cv::Point2f &ObjectBasicData::GetCenter() {
  // Making sure it is calculated.
  GetRotatedRect();
  return _rect.center;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetDistanceFromCenter() {
  // Makes sure it is calculated
  if (!_is_calculated_map[DISTANCE_FROM_CENTER]) {
    cv::Point center = GetCenter();
    float x_dist = abs(center.x - _original_image.cols / 2);
    float y_dist = abs(center.y - _original_image.rows / 2);
    _distance_from_center = x_dist * x_dist + y_dist + y_dist;
    _is_calculated_map[DISTANCE_FROM_CENTER] = true;
  }
  return _distance_from_center;
}

//-----------------------------------------------------------------------------
//
inline const cv::Rect &ObjectBasicData::GetUprightRect() {
  if (!_is_calculated_map[UP_RIGHT_RECT]) {
    _up_right_rect = cv::boundingRect(_contour);
    _is_calculated_map[UP_RIGHT_RECT] = true;
  }
  return _up_right_rect;
}

//-----------------------------------------------------------------------------
//
inline const cv::Mat ObjectBasicData::GetBinaryImageAtUprightRect() {
  // Making sure we have calculated the rectangle.
  cv::Rect uprightRect = GetUprightRect();
  // Clone is necessary since the object is created now
  // OpenCV Mat are smart pointer
  return cv::Mat(_binary_image, uprightRect).clone();
}

//-----------------------------------------------------------------------------
//
inline contour_t ObjectBasicData::GetContourCopy() { return _contour; }

//-----------------------------------------------------------------------------
//
inline const cv::Size ObjectBasicData::GetImageSize() {
  return _original_image.size();
}

//-----------------------------------------------------------------------------
//
inline const cv::Mat ObjectBasicData::GetBinaryImage() {
  return _original_image;
}

//-----------------------------------------------------------------------------
//
inline const cv::Mat ObjectBasicData::GetOriginalImage() {
  return _binary_image;
}

#endif
