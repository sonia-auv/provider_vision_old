/**
 * \file	object_basic_data.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_basic_data.h>

ObjectBasicData::ObjectBasicData(const cv::Mat &originalImage,
                                 const cv::Mat &binaryImage,
                                 const Contour &contour)
    : _area(0.0f),
      _convex_hull_area(0.0f),
      _circumference(0.0f),
      _planes(NB_OF_PLANE),
      _original_image(originalImage),
      _binary_image(binaryImage),
      _contour(contour),
      _areaRanking(0.0f),
      _lengthRanking(0.0f),
      _distance_from_center(10000.0f),
      _vote_count(0) {
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(AREA, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(CONVEX_HULL, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(CIRCUMFERENCE, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(ROTATED_RECT, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(UP_RIGHT_RECT, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(MOMENTS, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(PLANES, false));

  assert(!originalImage.empty());
  assert(!binaryImage.empty());
}

//-----------------------------------------------------------------------------
//
const cv::Mat &ObjectBasicData::GetPlanes(int planesID) {
  if (!_is_calculated_map[PLANES]) {
    cv::Mat gray, hsi;
    _planes.resize(ObjectBasicData::NB_OF_PLANE);
    cv::cvtColor(_original_image, gray, CV_BGR2GRAY);
    cv::cvtColor(_original_image, hsi, CV_BGR2HSV);
    // Set to zeros
    for (int i = 0; i < 7; i++)
      _planes[i] =
          cv::Mat::zeros(_original_image.rows, _original_image.cols, CV_8UC1);

    cv::split(_original_image, &_planes[0]);
    cv::split(hsi, &_planes[3]);
    gray.copyTo(_planes[6]);
    _is_calculated_map[PLANES] = true;
  }
  // Safety. Should be the constant set in this class, but...
  SetPlaneInRange(planesID);

  return _planes[planesID];
}

//-----------------------------------------------------------------------------
//
const cv::Moments &ObjectBasicData::GetMoments(bool useBinary) {
  if (!_is_calculated_map[MOMENTS]) {
    if (useBinary)
      _cv_moments = cv::moments(_binary_image, useBinary);
    else {
      cv::Mat gray;
      cv::cvtColor(_original_image, gray, CV_BGR2GRAY);
      _cv_moments = cv::moments(_binary_image, useBinary);
    }
    _is_calculated_map[MOMENTS] = true;
  }
  return _cv_moments;
}
