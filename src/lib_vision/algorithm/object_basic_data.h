/**
 * \file	object_basic_data.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Pierluc Bédard <pierlucbed@gmail.com>
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIB_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_
#define LIB_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_

#include <lib_vision/algorithm/rot_rect.h>
#include <lib_vision/algorithm/type_and_const.h>
#include "lib_vision/algorithm/contour.h"
#include <assert.h>

class ObjectBasicData {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectBasicData>;

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
    CONVEX_HULL,
    CIRCUMFERENCE,
    ROTATED_RECT,
    UP_RIGHT_RECT,
    MOMENTS,
    PLANES
  };

  ObjectBasicData(const cv::Mat &originalImage, const cv::Mat &binaryImage,
                  const Contour &contour);

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

  float GetConvexHullArea();

  float GetCircumference();

  const RotRect &GetRotatedRect();

  float GetAngle();

  cv::Point2f &GetCenter();

  const cv::Rect &GetUprightRect();

  const cv::Moments &GetMoments(bool useBinary);

  // Images are already reference in opencv...
  const cv::Mat &GetPlanes(int planesID);

  cv::Mat GetBinaryImageAtUprightRect();

  Contour GetContourCopy();

  cv::Size GetImageSize();

  const cv::Mat &GetBinaryImage();

  const cv::Mat &GetOriginalImage();

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
  Contour _contour;
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
    _area = cv::contourArea(_contour.Get(), false);
    _is_calculated_map[AREA] = true;
  }
  return _area;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetLength() {
  if (!_is_calculated_map[ROTATED_RECT]) {
    _rect = RotRect(_contour.Get());
    _is_calculated_map[ROTATED_RECT] = true;
  }
  return _rect.size.height;
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
    cv::convexHull(_contour.Get(), convexHull, false, true);
    _convex_hull_area = cv::contourArea(convexHull, false);
    _is_calculated_map[CONVEX_HULL] = true;
  }
  return _convex_hull_area;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetCircumference() {
  if (!_is_calculated_map[CIRCUMFERENCE]) {
    _circumference = cv::arcLength(_contour.Get(), true);
    _is_calculated_map[CIRCUMFERENCE] = true;
  }
  return _circumference;
}

//-----------------------------------------------------------------------------
//
inline const RotRect &ObjectBasicData::GetRotatedRect() {
  if (!_is_calculated_map[ROTATED_RECT]) {
    _rect = RotRect(_contour.Get());
    _is_calculated_map[ROTATED_RECT] = true;
  }
  return _rect;
}

//-----------------------------------------------------------------------------
//
inline float ObjectBasicData::GetAngle() {
  GetRotatedRect();
  return _rect.angle;
}

//-----------------------------------------------------------------------------
//
inline cv::Point2f &ObjectBasicData::GetCenter() {
  GetRotatedRect();
  return _rect.center;
}

//-----------------------------------------------------------------------------
//
inline const cv::Rect &ObjectBasicData::GetUprightRect() {
  if (!_is_calculated_map[UP_RIGHT_RECT]) {
    _up_right_rect = cv::boundingRect(_contour.Get());
    _is_calculated_map[UP_RIGHT_RECT] = true;
  }
  return _up_right_rect;
}

//-----------------------------------------------------------------------------
//
inline cv::Mat ObjectBasicData::GetBinaryImageAtUprightRect() {
  // Making sure we have calculated the rectangle.
  cv::Rect uprightRect = GetUprightRect();
  // Clone is necessary since the object is created now
  // OpenCV Mat are smart pointer
  return cv::Mat(_binary_image, uprightRect);
}

//-----------------------------------------------------------------------------
//
inline Contour ObjectBasicData::GetContourCopy() { return _contour; }

//-----------------------------------------------------------------------------
//
inline cv::Size ObjectBasicData::GetImageSize() {
  return _original_image.size();
}

//-----------------------------------------------------------------------------
//
inline const cv::Mat &ObjectBasicData::GetBinaryImage() {
  return _original_image;
}

//-----------------------------------------------------------------------------
//
inline const cv::Mat &ObjectBasicData::GetOriginalImage() {
  return _binary_image;
}

#endif // LIB_VISION_ALGORITHM_OBJECT_BASIC_DATA_H_
