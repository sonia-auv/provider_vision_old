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
    : _targetInited(false),
      _center(0, 0),
      _dimension(0, 0),
      _angle(0),
      _specialField_1(""),
      _specialField_2("") {}

//==============================================================
//
Target::~Target() {}

//==============================================================
//
void Target::setTarget(const Features &feat) {
  _targetInited = true;
  setCenter(feat.getOffsetCenter());
  setSize(feat.getWidth(), feat.getHeight());
  setAngle(feat.getAngle());
}

//==============================================================
//
void Target::setTarget(ObjectFullData::Ptr &obj) {
  _targetInited = true;
  RotRect rrect = obj->GetRotatedRect();
  setCenter( rrect.center.x, rrect.center.y );
  setCameraOffset(&_center, obj->GetImageSize().height, obj->GetImageSize().width);
  setSize(rrect.size.height , rrect.size.width);
  setAngle(rrect.angle);
}

//==============================================================
//
void Target::setTarget(int x, int y, float width, float height,
                       float angleTarget, const std::string &spec_field_1,
                       const std::string &spec_field_2) {
  _targetInited = true;
  setCenter(x, y);
  setSize(width, height);
  setAngle(angleTarget);
  setSpecField_1(spec_field_1);
  setSpecField_2(spec_field_2);
}

//==============================================================
//
std::string Target::outputString() {
  std::stringstream stringOutput;
  if (_targetInited)
    stringOutput << _center.x << ',' << _center.y << ',' << _dimension.width
                 << ',' << _dimension.height << ',' << _angle << ','
                 << _specialField_1 << "," << _specialField_2;
  stringOutput << ';';
  return stringOutput.str();
}
