/**
 * \file	targer.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/target.h>

//==============================================================
//
Target::Target()
    : target_is_inited_(false),
      center_(0, 0),
      dimension_(0, 0),
      angle_(0),
      special_field_1_(""),
      special_field_2_("") {}

//==============================================================
//
Target::~Target() {}

//==============================================================
//
void Target::SetTarget(ObjectFullData::Ptr obj) {
  target_is_inited_ = true;
  RotRect rrect = obj->GetRotatedRect();
  SetCenter(rrect.center.x, rrect.center.y);
  setCameraOffset(&center_, obj->GetImageSize().height,
                  obj->GetImageSize().width);
  SetSize(rrect.size.height, rrect.size.width);
  SetAngle(rrect.angle);
}

//==============================================================
//
void Target::SetTarget(int x, int y, float width, float height,
                       float angleTarget, const std::string &spec_field_1,
                       const std::string &spec_field_2) {
  target_is_inited_ = true;
  SetCenter(x, y);
  SetSize(width, height);
  SetAngle(angleTarget);
  SetSpecField_1(spec_field_1);
  SetSpecField_2(spec_field_2);
}

//==============================================================
//
std::string Target::OutputString() {
  std::stringstream stringOutput;
  if (target_is_inited_)
    stringOutput << center_.x << ',' << center_.y << ',' << dimension_.width
                 << ',' << dimension_.height << ',' << angle_ << ','
                 << special_field_1_ << "," << special_field_2_;
  stringOutput << ';';
  return stringOutput.str();
}
