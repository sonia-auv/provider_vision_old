/**
 * \file	object_frame_memory.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_OBJECT_FRAME_MEMORY_
#define VISION_FILTER_OBJECT_FRAME_MEMORY_

#include <CLCircularVector.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/general_function.h>
#include <vector>

class ObjectFrameMemory {
 public:
  ObjectFrameMemory(unsigned int memorySize);

  ~ObjectFrameMemory() {}

  // When getting object in the past, we compare the center
  // and the ratio to make sure it stills fit the same object.
  // If the ratio difference is smaller than RATIO_MAX_DIFFERENCE
  // we consider it as good object.
  static const float DISTANCE_MAX_DIFFERENCE;
  static const float RATIO_MAX_DIFFERENCE;

  void AddFrameObjects(ObjectFullData::FullObjectPtrVec &objectVector);

  unsigned int GetMemorySize();

  // Use the center and the ratio to find an object in the past object list.
  ObjectFullData::FullObjectPtrVec GetPastObjectsViaCenter(
      const cv::Point &center, const float objectRatio);

 private:
  CLCircularVector<ObjectFullData::FullObjectPtrVec> _previous_frames;
  unsigned int _memory_size;
};

inline unsigned int ObjectFrameMemory::GetMemorySize() { return _memory_size; }

#endif
