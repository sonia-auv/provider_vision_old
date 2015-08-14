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
  _previous_frames.AddElement(objectVector);
}

//-----------------------------------------------------------------------------
//
ObjectFullData::FullObjectPtrVec ObjectFrameMemory::GetPastObjectsViaCenter(
    const cv::Point &center, const float objectRatio) {
  ObjectFullData::FullObjectPtrVec objVec;

  // For i frame
  for (int i = 0, buffSize = _previous_frames.size(); i < buffSize; i++) {
    float shortestDistance = 100000.0f;
    ObjectFullData::Ptr nearestObject = NULL;
    ObjectFullData::FullObjectPtrVec currentFrameData =
        _previous_frames.GetSpecificElement(i);

    // for all the object in the frame
    for (int j = 0, size = currentFrameData.size(); j < size; j++) {
      ObjectFullData::Ptr analysedObject = currentFrameData[j];
      if (analysedObject.IsNotNull()) {
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
    if (nearestObject.IsNotNull()) {
      objVec.push_back(nearestObject);
    }
  }
  return objVec;
}

//=============================================================================
//=============================================================================
//=============================================================================
//=========================== UNIT TEST AREA ==================================
//=============================================================================
//=============================================================================
//=============================================================================
#include <TCUnitTest.h>
#include <lib_vision/algorithm/general_function.h>
#include <opencv2/opencv.hpp>

TC_DEFINE_UNIT_TEST(ObjectFrameMemoryUT) {
  printf("Starting unit test on ObjectFrameMemory\n");

  contourList_t contours;
  cv::Mat binaryImage(1000, 1000, CV_8UC1, cv::Scalar::all(0));
  cv::Mat originalImage(1000, 1000, CV_8UC3, cv::Scalar::all(0));

  // square contour 100 of area, 1 of ratio
  // 100 filled, convexity 0 , circularity ?
  // last in area rank and length rank
  std::vector<cv::Point> tmpContour;
  tmpContour.push_back(cv::Point(0, 0));
  tmpContour.push_back(cv::Point(0, 10));
  tmpContour.push_back(cv::Point(10, 10));
  tmpContour.push_back(cv::Point(10, 0));
  contours.push_back(tmpContour);
  tmpContour.clear();

  // M like contour.
  tmpContour.push_back(cv::Point(100, 100));
  tmpContour.push_back(cv::Point(200, 100));
  tmpContour.push_back(cv::Point(150, 300));
  tmpContour.push_back(cv::Point(200, 500));
  tmpContour.push_back(cv::Point(100, 500));
  contours.push_back(tmpContour);
  tmpContour.clear();

  tmpContour.push_back(cv::Point(400, 700));
  tmpContour.push_back(cv::Point(750, 700));
  tmpContour.push_back(cv::Point(750, 800));
  tmpContour.push_back(cv::Point(400, 800));
  contours.push_back(tmpContour);
  tmpContour.clear();
  cv::drawContours(originalImage, contours, -1, CV_RGB(255, 0, 255), -1);
  cv::cvtColor(originalImage, binaryImage, CV_BGR2GRAY);

  ObjectFullData::FullObjectPtrVec tmp;
  tmp.push_back(ObjectFullData::Ptr(
      new ObjectFullData(originalImage, binaryImage, contours[0])));
  tmp.push_back(ObjectFullData::Ptr(
      new ObjectFullData(originalImage, binaryImage, contours[1])));
  tmp.push_back(ObjectFullData::Ptr(
      new ObjectFullData(originalImage, binaryImage, contours[2])));

  ObjectFrameMemory frameMemory(3);
  // Fill history buffer.
  frameMemory.AddFrameObjects(tmp);
  frameMemory.AddFrameObjects(tmp);
  frameMemory.AddFrameObjects(tmp);

  TC_COMMENT("Testing retrieval by center and ratio")

  /// Split the M shape to prevent retrieval of the object via center.
  // We create a frame where we should miss the M form.
  cv::Mat tmpBinary, tmpOriginal;
  tmpOriginal = originalImage.clone();
  cv::rectangle(tmpOriginal, cv::Point(100, 290), cv::Point(200, 310),
                CV_RGB(0, 0, 0), -1);
  cv::cvtColor(tmpOriginal, tmpBinary, CV_BGR2GRAY);
  contourList_t contoursTemp;
  retrieveAllContours(tmpBinary, contoursTemp);
  TC_TEST_FAIL("Found four contours", contoursTemp.size() == 4)
  // Push faulty object
  ObjectFullData::FullObjectPtrVec tmp2;
  for (int i = 0; i < contoursTemp.size(); i++) {
    tmp2.push_back(ObjectFullData::Ptr(
        new ObjectFullData(originalImage, binaryImage, contoursTemp[i])));
  }

  // Add the frame where the M form is missing
  frameMemory.AddFrameObjects(tmp2);

  // Add another frame where all the object are ok.
  // Update with real contour to find it in the past objects
  frameMemory.AddFrameObjects(tmp);

  // Here we altered only the biggest object in time, so the one with the
  // current
  // highest area rating should have only 2 ancestor, since one of them was
  // splitted
  int objectSum = 0;
  for (int i = 0, size = tmp.size(); i < size; i++) {
    ObjectFullData::Ptr tmpObj = tmp[i];
    ObjectFullData::FullObjectPtrVec vec = frameMemory.GetPastObjectsViaCenter(
        tmpObj->GetCenter(), tmpObj->GetRatio());
    cv::Point center = tmpObj->GetCenter();
    float ratio = tmpObj->GetRatio();
    objectSum += vec.size();
  }
  // We have 3 contours in tmp. In the past, one of them ( the M )
  // has been split. Therefor, frame memory should not be able to find it
  // since the center is not there AND the ratio is not good. So we have 3 form,
  // in 3 frame
  // but one of them is missing so 3*3 - 1 = 8 retrieve form in the past.
  TC_TEST_FAIL("Founded ok", objectSum == 8);
  printf("System all clear and good to go");
  return true;
}
TC_END_UNIT_TEST(ObjectFrameMemoryUT);
