/**
 * \file	feature_factory.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_FEATURE_FACTORY_H_
#define VISION_FILTER_FEATURE_FACTORY_H_

#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/object_ranker.h>
#include <lib_vision/algorithm/object_frame_memory.h>
#include <vector>

class FeatureFactory {
 public:
  enum FEATURE_TYPE {
    AREA_RANK,
    LENGTH_RANK,
    RATIO,
    CONVEXITY,
    PERCENT_FILLED,
    CIRCULARITY,
    PRESENCE_CONSISTENCY,
    HUE_MEAN
  };

  FeatureFactory(unsigned int memorySize);

  ~FeatureFactory(){};

  void CalculateFeatureVectors(ObjectFullData::FullObjectPtrVec objects);

  void SetFeatureToCompute(std::vector<FEATURE_TYPE> featureToCompute);

  void SetAllFeatureToCompute();

  // feature funcitions
  // For the ranking, require to call ObjectRanking first
  float AreaFeature(ObjectFullData::Ptr object);

  float LengthFeature(ObjectFullData::Ptr object);

  float RatioFeature(ObjectFullData::Ptr object);

  float ConvexityFeature(ObjectFullData::Ptr object);

  float PercentFilledFeature(ObjectFullData::Ptr object);

  float CircularityFeature(ObjectFullData::Ptr object);

  float PresenceConsistencyFeature(ObjectFullData::Ptr object);

  float HueMeanFeature(ObjectFullData::Ptr object);

 private:
  // define the type "Pointer to method to calculate a feature"
  typedef float (FeatureFactory::*FEATURE_CALCULATOR_PTR)(ObjectFullData::Ptr);

  // the vector of function enables iterating through the function that needs
  // to be call in odrer to compute the good feature.
  std::vector<FEATURE_CALCULATOR_PTR> _feature_calculator_vector;

  ObjectFrameMemory _frame_memory;

  // Since we only need to do it once per frame, we do it before
  // the loop.
  bool _calculate_area_rank, _calculate_length_rank;
};

//=============================================================================
//	INLINE
//-----------------------------------------------------------------------------
//
inline float FeatureFactory::AreaFeature(ObjectFullData::Ptr object) {
  float areaRank = 0.0f;
  if (object.IsNotNull()) {
    areaRank = object->GetAreaRank();
  }
  return areaRank;
}

//-----------------------------------------------------------------------------
//
inline float FeatureFactory::LengthFeature(ObjectFullData::Ptr object) {
  float lengthRank = 0.0f;
  if (object.IsNotNull()) {
    lengthRank = object->GetLengthRank();
  }
  return lengthRank;
}

//-----------------------------------------------------------------------------
//
inline float FeatureFactory::RatioFeature(ObjectFullData::Ptr object) {
  float ratio = 0.0f;
  if (object.IsNotNull()) {
    ratio = object->GetRatio();
  }
  return ratio;
}

//-----------------------------------------------------------------------------
//
inline float FeatureFactory::ConvexityFeature(ObjectFullData::Ptr object) {
  float convexity = 0.0f;
  if (object.IsNotNull()) {
    // safety, should not happen
    float convexHull = object->GetConvexHullArea();
    if (convexHull > 0)
      convexity = object->GetArea() / object->GetConvexHullArea();
  }
  return convexity;
}

//-----------------------------------------------------------------------------
//
inline float FeatureFactory::CircularityFeature(ObjectFullData::Ptr object) {
  float circularity = 0.0f;
  if (object.IsNotNull()) {
    // Here we use pow on radius instead of sqrt on area because
    // pow is less hard computation
    float radiusCircum = pow(object->GetCircumference() / (2 * M_PI), 2);
    float radiusArea = object->GetArea() / (M_PI);
    if (radiusCircum != 0 && radiusArea != 0) {
      circularity = radiusCircum > radiusArea ? radiusArea / radiusCircum
                                              : radiusCircum / radiusArea;
    }
  }
  return circularity;
}

//-----------------------------------------------------------------------------
//
inline float FeatureFactory::PresenceConsistencyFeature(
    ObjectFullData::Ptr object) {
  float consistency = 0.0f;
  if (object.IsNotNull()) {
    ObjectFullData::FullObjectPtrVec vec =
        _frame_memory.GetPastObjectsViaCenter(object->GetCenter(),
                                              object->GetRatio());
    consistency = float(vec.size()) / float(_frame_memory.GetMemorySize());
  }
  return consistency;
}

#endif  //_FEATURE_FACTORY_H_
