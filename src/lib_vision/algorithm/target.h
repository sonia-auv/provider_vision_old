/**
 * \file	target.h
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

#ifndef LIB_VISION_ALGORITHM_TARGET_H_
#define LIB_VISION_ALGORITHM_TARGET_H_

#include <memory>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/general_function.h>

class Target {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Target>;

  //============================================================================
  // P U B L I C   C / D T O R S

  Target();

  ~Target();

  //============================================================================
  // P U B L I C   M E T H O D S

  void SetTarget(int x, int y, float width, float height, float angle,
                 const std::string &spec_field_1 = "",
                 const std::string &spec_field_2 = "");

  // WILL USE OFFSETED CENTER !!!
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
  //============================================================================
  // P R I V A T E   M E M B E R S

  // To know if we should put the data in the output or return empty
  // descriptor
  bool target_is_inited_;

  cv::Point center_;

  cv::Size dimension_;

  float angle_;

  // Bins name, buoy colors, etc.
  std::string special_field_1_;

  std::string special_field_2_;
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

#endif  // LIB_VISION_ALGORITHM_TARGET_H_
