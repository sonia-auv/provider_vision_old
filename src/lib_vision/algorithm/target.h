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

  void SetTarget(int x, int y, float width, float height, float angle,
                 const std::string &spec_field_1 = "",
                 const std::string &spec_field_2 = "");

  // WILL USE OFFSETED CENTER !!!
  void SetTarget(const Features &feat);
  void SetTarget(ObjectFullData::Ptr obj);

  void SetCenter(int x, int y);

  void SetCenter(const cv::Point &pt);

  void SetSize(int width, int height);

  void SetSize(const cv::Size &sz);

  void SetAngle(float angle);

  void SetSpecField_1(const std::string &field);

  void SetSpecField_2(const std::string &field);

  void SetSpecFields(const std::string &field1, const std::string &field2);

  cv::Point GetTarget();
  std::string GetSpecField_1();
  std::string GetSpecField_2();

  std::string OutputString();

 private:
  // To know if we should put the data in the output or return empty
  // descriptor
  bool target_is_inited_;
  cv::Point center_;
  cv::Size dimension_;
  float angle_;
  // Bins name, buoy colors, etc.
  std::string special_field_1_, special_field_2_;
};

//=============================================================================
//
inline void Target::SetCenter(int x, int y) {
  center_.x = x;
  center_.y = y;
}

//=============================================================================
//
inline void Target::SetCenter(const cv::Point &pt) { center_ = pt; }

//=============================================================================
//
inline void Target::SetSize(int width, int height) {
  dimension_.width = width;
  dimension_.height = height;
}

//=============================================================================
//
inline void Target::SetSize(const cv::Size &sz) { dimension_ = sz; }

//=============================================================================
//
inline void Target::SetAngle(float angle) { angle_ = angle; }

//=============================================================================
//
inline void Target::SetSpecField_1(const std::string &field) {
  special_field_1_ = field;
}

//=============================================================================
//
inline void Target::SetSpecField_2(const std::string &field) {
  special_field_2_ = field;
}

//=============================================================================
//
inline void Target::SetSpecFields(const std::string &field1,
                                  const std::string &field2) {
  special_field_1_ = field1;
  special_field_2_ = field2;
}

inline cv::Point Target::GetTarget() { return center_; }
inline std::string Target::GetSpecField_1() { return special_field_1_; };
inline std::string Target::GetSpecField_2() { return special_field_2_; };

#endif
