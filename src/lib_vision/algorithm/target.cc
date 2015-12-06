/**
 * \file	target.cc
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

#include <lib_vision/algorithm/target.h>

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
Target::Target()
    : target_is_inited_(false),
      center_(0, 0),
      dimension_(0, 0),
      angle_(0),
      special_field_1_(""),
      special_field_2_("") {}

//------------------------------------------------------------------------------
//
Target::~Target() {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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
