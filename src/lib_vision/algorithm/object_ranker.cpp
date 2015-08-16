/**
 * \file	object_ranker.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_ranker.h>

//-----------------------------------------------------------------------------
//
void ObjectRanker::RankByArea(ObjectFullData::FullObjectPtrVec objects) {
  std::sort(objects.begin(), objects.end(), ObjectRanker::AreaSortFunction);
  for (int i = 0, size = objects.size(); i < size; i++) {
    if (objects[i].get() != nullptr) {
      objects[i]->SetAreaRank((float(size - i)) / float(size));
    }
  }
}

//-----------------------------------------------------------------------------
//
void ObjectRanker::RankByLength(ObjectFullData::FullObjectPtrVec objects) {
  std::sort(objects.begin(), objects.end(), ObjectRanker::LengthSortFunction);
  for (int i = 0, size = objects.size(); i < size; i++) {
    if (objects[i].get() != nullptr) {
      objects[i]->SetLengthRank(float((size - i)) / float(size));
    }
  }
}
