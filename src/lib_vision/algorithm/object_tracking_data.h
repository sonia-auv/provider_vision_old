/**
 * \file	object_tracking_data.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_OBJECT_TRACKING_DATA_H_
#define VISION_FILTER_OBJECT_TRACKING_DATA_H_

// OBjectTrackingData is a basic container class that holds information
// about an object's past state in time. EX. In the past X frames, how
// many time was the object present? Of How much the area or ratio changed
// in the past frames?
class ObjectTrackingData {
 public:
  ObjectTrackingData () : _presence_count(0.0f){};

  virtual ~ObjectTrackingData (){};

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
