/**
 * \file	object_frame_memory.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_frame_memory.h>

//=============================================================================
//	CONSTANT
// 20 pix offset center should cover small noise error and small
// displacement.
const float ObjectFrameMemory::DISTANCE_MAX_DIFFERENCE = 20;
// A difference of 10% of ratio is big enough to discard an object.
const float ObjectFrameMemory::RATIO_MAX_DIFFERENCE = 0.1;

//=============================================================================
//	CONSTRUCTOR AND DESTRUCTOR
//-----------------------------------------------------------------------------
//
ObjectFrameMemory::ObjectFrameMemory(unsigned int memorySize)
    : _previous_frames(memorySize), _memory_size(memorySize) {}

//=============================================================================
//	METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
void ObjectFrameMemory::AddFrameObjects(
    ObjectFullData::FullObjectPtrVec &objectVector) {
  _previous_frames.push_back(objectVector);
}

//-----------------------------------------------------------------------------
//
ObjectFullData::FullObjectPtrVec ObjectFrameMemory::GetPastObjectsViaCenter(
    const cv::Point &center, const float objectRatio) {
  ObjectFullData::FullObjectPtrVec objVec;

  // For i frame
  for (int i = 0, buffSize = _previous_frames.size(); i < buffSize; i++) {
    float shortestDistance = 100000.0f;
    std::shared_ptr<ObjectFullData> nearestObject = NULL;
    ObjectFullData::FullObjectPtrVec currentFrameData = _previous_frames.at(i);

    // for all the object in the frame
    for (int j = 0, size = currentFrameData.size(); j < size; j++) {
      std::shared_ptr<ObjectFullData> analysedObject = currentFrameData[j];
      if (analysedObject.get() != nullptr) {
        cv::Point analysedCenter = analysedObject->GetCenter();
        float distance = eucledianPointDistance(center, analysedCenter);

        float analysedRatio = analysedObject->GetRatio();
        float ratioDifference = fabsf(analysedRatio - objectRatio);

        if (distance < shortestDistance && distance < DISTANCE_MAX_DIFFERENCE &&
            ratioDifference < RATIO_MAX_DIFFERENCE) {
          shortestDistance = distance;
          nearestObject = analysedObject;
        }
      }
    }
    if (nearestObject.get() != nullptr) {
      objVec.push_back(nearestObject);
    }
  }
  return objVec;
}
