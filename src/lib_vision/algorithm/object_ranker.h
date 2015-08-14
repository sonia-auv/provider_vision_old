/**
 * \file	object_ranker.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_OBJECT_RANKER_H_
#define VISION_FILTER_OBJECT_RANKER_H_

#include <lib_vision/algorithm/object_full_data.h>

// Class that simply rank the object
// with different value.
class ObjectRanker {
 public:
  // sort and set the value in each objects.
  static void RankByArea(ObjectFullData::FullObjectPtrVec objects);

  static void RankByLength(ObjectFullData::FullObjectPtrVec objects);

  // Function for std::sort function
  static bool AreaSortFunction(ObjectFullData::Ptr a, ObjectFullData::Ptr b);

  static bool LengthSortFunction(ObjectFullData::Ptr a, ObjectFullData::Ptr b);
};

//-----------------------------------------------------------------------------
//
inline bool ObjectRanker::AreaSortFunction(ObjectFullData::Ptr a,
                                           ObjectFullData::Ptr b) {
  if (a.IsNotNull() && b.IsNotNull()) {
    return a->GetArea() > b->GetArea();
  }
  return false;
}

//-----------------------------------------------------------------------------
//
inline bool ObjectRanker::LengthSortFunction(ObjectFullData::Ptr a,
                                             ObjectFullData::Ptr b) {
  if (a.IsNotNull() && b.IsNotNull()) {
    return a->GetLength() > b->GetLength();
  }
  return false;
}

#endif
