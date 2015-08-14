/**
 * \file	target.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_TARGET_H_
#define VISION_FILTER_TARGET_H_

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <lib_vision/algorithm/features.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/general_function.h>

class Target {
 public:
  Target();

  ~Target();

  void setTarget(int x, int y, float width, float height, float angle,
                 const std::string &spec_field_1 = "",
                 const std::string &spec_field_2 = "");

  // WILL USE OFFSETED CENTER !!!
  void setTarget(const Features &feat);
  void setTarget(ObjectFullData::Ptr &obj);

  void setCenter(int x, int y);

  void setCenter(const cv::Point &pt);

  void setSize(int width, int height);

  void setSize(const cv::Size &sz);

  void setAngle(float angle);

  void setSpecField_1(const std::string &field);

  void setSpecField_2(const std::string &field);

  void setSpecFields(const std::string &field1, const std::string &field2);

  cv::Point getTarget();
  std::string getSpecField_1();
  std::string getSpecField_2();

  std::string outputString();

 private:
  // To know if we should put the data in the output or return empty
  // descriptor
  bool _targetInited;
  cv::Point _center;
  cv::Size _dimension;
  float _angle;
  // Bins name, buoy colors, etc.
  std::string _specialField_1, _specialField_2;
};

//=============================================================================
//
inline void Target::setCenter(int x, int y) {
  _center.x = x;
  _center.y = y;
}

//=============================================================================
//
inline void Target::setCenter(const cv::Point &pt) { _center = pt; }

//=============================================================================
//
inline void Target::setSize(int width, int height) {
  _dimension.width = width;
  _dimension.height = height;
}

//=============================================================================
//
inline void Target::setSize(const cv::Size &sz) { _dimension = sz; }

//=============================================================================
//
inline void Target::setAngle(float angle) { _angle = angle; }

//=============================================================================
//
inline void Target::setSpecField_1(const std::string &field) {
  _specialField_1 = field;
}

//=============================================================================
//
inline void Target::setSpecField_2(const std::string &field) {
  _specialField_2 = field;
}

//=============================================================================
//
inline void Target::setSpecFields(const std::string &field1,
                                  const std::string &field2) {
  _specialField_1 = field1;
  _specialField_2 = field2;
}

inline cv::Point Target::getTarget() { return _center; }
inline std::string Target::getSpecField_1(){return _specialField_1;};
inline std::string Target::getSpecField_2(){return _specialField_2;};

#endif
