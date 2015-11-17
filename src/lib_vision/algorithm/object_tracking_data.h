/**
 * \file	object_tracking_data.h
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

#ifndef VISION_FILTER_OBJECT_TRACKING_DATA_H_
#define VISION_FILTER_OBJECT_TRACKING_DATA_H_

// OBjectTrackingData is a basic container class that holds information
// about an object's past state in time. EX. In the past X frames, how
// many time was the object present? Of How much the area or ratio changed
// in the past frames?
class ObjectTrackingData {
 public:
  ObjectTrackingData() : _presence_count(0.0f){};

  virtual ~ObjectTrackingData(){};

  void SetPresenceCount(float presenceCount);

  float GetPresenceCount();

 private:
  // In percent, nb of presence/ nb of frame in memory
  float _presence_count;
  // In percent, the variation of the ratio value
  // float _ratio_variation;
};

//=============================================================================
//	INLINE METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
inline void ObjectTrackingData::SetPresenceCount(float presenceCount) {
  _presence_count = presenceCount;
}

//-----------------------------------------------------------------------------
//
inline float ObjectTrackingData::GetPresenceCount() { return _presence_count; }

#endif
