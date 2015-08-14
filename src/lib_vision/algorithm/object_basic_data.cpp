/**
 * \file	object_basic_data.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_basic_data.h>

ObjectBasicData::ObjectBasicData(const cv::Mat &originalImage,
                                 const cv::Mat &binaryImage,
                                 const contour_t &contour)
    : _area(0.0f),
      _convex_hull_area(0.0f),
      _circumference(0.0f),
      _planes(NB_OF_PLANE),
      _original_image(originalImage),
      _binary_image(binaryImage),
      _contour(contour),
      _areaRanking(0.0f),
      _lengthRanking(0.0f),
      _distance_from_center(10000.0f),
      _vote_count(0){
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(AREA, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(CONVEX_HULL, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(CIRCUMFERENCE, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(ROTATED_RECT, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(UP_RIGHT_RECT, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(MOMENTS, false));
  _is_calculated_map.insert(std::pair<OBJECT_DATA, bool>(PLANES, false));

  assert(!originalImage.empty());
  assert(!binaryImage.empty());
}

//-----------------------------------------------------------------------------
//
const cv::Mat ObjectBasicData::GetPlanes(int planesID) {
  if (!_is_calculated_map[PLANES]) {
    cv::Mat gray, hsi;
    _planes.resize(ObjectBasicData::NB_OF_PLANE);
    cv::cvtColor(_original_image, gray, CV_BGR2GRAY);
    cv::cvtColor(_original_image, hsi, CV_BGR2HSV);
    // Set to zeros
    for (int i = 0; i < 7; i++)
      _planes[i] =
          cv::Mat::zeros(_original_image.rows, _original_image.cols, CV_8UC1);

    cv::split(_original_image, &_planes[0]);
    cv::split(hsi, &_planes[3]);
    gray.copyTo(_planes[6]);
    _is_calculated_map[PLANES] = true;
  }
  // Safety. Should be the constant set in this class, but...
  SetPlaneInRange(planesID);

  return _planes[planesID];
}

//-----------------------------------------------------------------------------
//
const cv::Moments &ObjectBasicData::GetMoments(bool useBinary) {
  if (!_is_calculated_map[MOMENTS]) {
    if (useBinary)
      _cv_moments = cv::moments(_binary_image, useBinary);
    else {
      cv::Mat gray;
      cv::cvtColor(_original_image, gray, CV_BGR2GRAY);
      _cv_moments = cv::moments(_binary_image, useBinary);
    }
    _is_calculated_map[MOMENTS] = true;
  }
  return _cv_moments;
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

bool CompareMoments(cv::Moments moment1, cv::Moments moment2) {
  return moment1.m00 == moment2.m00 && moment1.m01 == moment2.m01 &&
         moment1.m02 == moment2.m02 && moment1.m03 == moment2.m03 &&
         moment1.m10 == moment2.m10 && moment1.m20 == moment2.m20 &&
         moment1.m30 == moment2.m30;
}

TC_DEFINE_UNIT_TEST(ObjectBasicDataUT) {
  printf("Starting unit test on ObjectBasicData");

  cv::Mat unitTestImage, binaryImage;
  std::string pathToFile(getenv("SONIA_WORKSPACE_ROOT"));
  pathToFile += "/vision_filter/library/doc/BasicObjectUnitTest.png";
  unitTestImage = cv::imread(pathToFile);
  TC_TEST_FAIL("Image loaded", !unitTestImage.empty());

  if (unitTestImage.channels() != 1) {
    cv::cvtColor(unitTestImage, binaryImage, CV_BGR2GRAY);
  }
  contourList_t contours;
  hierachy_t hierachy;
  retrieveAllContours(binaryImage, contours);

  std::vector<ObjectBasicData> objectData;

  for (int i = 0; i < contours.size(); i++) {
    objectData.push_back(
        ObjectBasicData(unitTestImage, binaryImage, contours[i]));
  }

  TC_TEST_FAIL("All contour are found.", objectData.size() == 3);
  unitTestImage = cv::Mat::zeros(100, 100, CV_8UC3);
  for (int i = 0; i < objectData.size(); i++) {
    TC_TEST_FAIL("Object has is own copy of the image",
                 objectData[i]._original_image.cols != 100 &&
                     objectData[i]._original_image.rows != 100);

    float area = objectData[i].GetArea();
    TC_TEST_FAIL("Area ok", area > 0.0f && objectData[i]._area == area);

    float convexityArea = objectData[i].GetConvexHullArea();
    TC_TEST_FAIL("convex hull area ok",
                 convexityArea > 0.0f &&
                     objectData[i]._convex_hull_area == convexityArea);

    float circumference = objectData[i].GetCircumference();
    TC_TEST_FAIL(
        "circumference ok",
        circumference > 0.0f && objectData[i]._circumference == circumference);

    cv::Moments moments = objectData[i].GetMoments(true);
    TC_TEST_FAIL("moment ok",
                 CompareMoments(objectData[i]._cv_moments, moments));

    RotRect rrect = objectData[i].GetRotatedRect();
    TC_TEST_FAIL("rotated rect ok", rrect == objectData[i]._rect);

    cv::Rect uprightRect = objectData[i].GetUprightRect();
    TC_TEST_FAIL("rotated rect ok",
                 uprightRect == objectData[i]._up_right_rect);

    cv::Mat plane = objectData[i].GetPlanes(ObjectBasicData::BLUE_PLANE);
    TC_TEST_FAIL("Planes okay", !plane.empty());

    TC_TEST_DIE(objectData[i]._planes.size() == ObjectBasicData::NB_OF_PLANE);

    for (int j = 0; j < ObjectBasicData::NB_OF_PLANE; j++) {
      printf("%d %d\n", objectData[i]._planes[j].rows,
             objectData[i]._planes[j].cols);
      // TC_TEST_FAIL("Plane all set", !objectData[i]._planes[i].empty());
    }

    std::map<ObjectBasicData::OBJECT_DATA, bool>::iterator iter =
        objectData[i]._is_calculated_map.begin();

    for (; iter != objectData[i]._is_calculated_map.end(); iter++) {
      TC_TEST_FAIL("Everything is calculated", (*iter).second);
    }
  }

  printf("System all clear and good to go!\n");

  return true;
}
TC_END_UNIT_TEST(ObjectBasicDataUT);
