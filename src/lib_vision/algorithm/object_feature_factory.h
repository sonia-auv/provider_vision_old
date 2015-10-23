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

#include <vector>
#include <map>
#include <memory>

#include "lib_vision/algorithm/object_full_data.h"
#include "lib_vision/algorithm/object_frame_memory.h"
#include "lib_vision/algorithm/object_feature.h"

class ObjectFeatureFactory {
 public:

  ObjectFeatureFactory(unsigned int memorySize);

  ~ObjectFeatureFactory() { };

  void ComputeAllFeature(ObjectFullData::FullObjectPtrVec objects);
  void ComputeAllFeature(ObjectFullData::Ptr object);

  void ComputeSelectedFeature(ObjectFullData::FullObjectPtrVec objects,
                             const std::vector<ObjectFeatureData::Feature> &feature);
  void ComputeSelectedFeature(ObjectFullData::Ptr object,
                             const std::vector<ObjectFeatureData::Feature> &feature);

 private:
  // feature funcitions
  // For the ranking, require to call ObjectRanking first
  static void RatioFeature
      (ObjectFullData::Ptr object);
  static void ConvexityFeature
      (ObjectFullData::Ptr object);
  static void PercentFilledFeature
      (ObjectFullData::Ptr object);
  static void CircularityFeature
      (ObjectFullData::Ptr object);
  static void PresenceConsistencyFeature
      (ObjectFullData::Ptr object);
  static void HueMeanFeature
      (ObjectFullData::Ptr object);
  static void SatMeanFeature
      (ObjectFullData::Ptr object);
  static void IntensityMeanFeature
      (ObjectFullData::Ptr object);
  static void RedMeanFeature
      (ObjectFullData::Ptr object);
  static void GreenMeanFeature
      (ObjectFullData::Ptr object);
  static void BlueMeanFeature
      (ObjectFullData::Ptr object);
  static void GrayMeanFeature
      (ObjectFullData::Ptr object);

  float CalculatePlaneMean
      (ObjectFullData::Ptr object, int plane);
 private:
  // define the type "Pointer to method to calculate a feature"
  typedef std::function<void(ObjectFullData::Ptr)> FeatureFunction;

  // the vector of function enables iterating through the function that needs
  // to be call in odrer to compute the good feature.
  std::map<ObjectFeatureData::Feature, FeatureFunction> feature_fct_map;

  ObjectFrameMemory _frame_memory;
};

//=============================================================================
//	INLINE
//-----------------------------------------------------------------------------
//
inline void
ObjectFeatureFactory::ComputeAllFeature(ObjectFullData::FullObjectPtrVec objects) {
  for (auto &object : objects) {
    ComputeAllFeature(object);
  }
}

//-----------------------------------------------------------------------------
//
inline void
ObjectFeatureFactory::ComputeAllFeature(ObjectFullData::Ptr object) {
  for (auto &fct : feature_fct_map) {
    (fct.second)(object);
  }
}

//-----------------------------------------------------------------------------
//
inline void
ObjectFeatureFactory::ComputeSelectedFeature(ObjectFullData::FullObjectPtrVec objects,
                                             const std::vector<ObjectFeatureData::Feature> &feature) {
  for (auto &object : objects) {
    ComputeSelectedFeature(object, feature);
  }

}

//-----------------------------------------------------------------------------
//
inline void
ObjectFeatureFactory::ComputeSelectedFeature(ObjectFullData::Ptr object,
                                             const std::vector<ObjectFeatureData::Feature> &feature) {
  for (const auto &feat : feature) {
    (feature_fct_map[feat])(object);
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::RatioFeature(
    ObjectFullData::Ptr object) {
  if (object) {
    RotRect rect = object->GetRotatedRect();
    object->SetRatio(rect.size.width / rect.size.height);
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::ConvexityFeature(
    ObjectFullData::Ptr object) {
  if (object) {
    // safety, should not happen
    float convexHull = object->GetConvexHullArea();
    if (convexHull > 0)
      object->SetConvexity(
          object->GetArea() / object->GetConvexHullArea());
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::CircularityFeature(
    ObjectFullData::Ptr object) {
  if (object) {
    // Here we use pow on radius instead of sqrt on area because
    // pow is less hard computation
    float radiusCircum = pow(object->GetCircumference() / (2 * M_PI), 2);
    float radiusArea = object->GetArea() / (M_PI);
    if (radiusCircum != 0 && radiusArea != 0) {
      object->SetConvexity(
          radiusCircum > radiusArea ? radiusArea / radiusCircum
                                    : radiusCircum / radiusArea);
    }
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::PresenceConsistencyFeature(
    ObjectFullData::Ptr object) {
  if (object) {
    float ratio = object->GetRatio();
    if (ratio == -1.0f) {
      RatioFeature(object);
      ratio = object->GetRatio();
    }
    ObjectFullData::FullObjectPtrVec vec =
        _frame_memory.GetPastObjectsViaCenter(object->GetCenter(), ratio);
    object->SetPresenceConsistency(
        float(vec.size()) / float(_frame_memory.GetMemorySize()));
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::HueMeanFeature(ObjectFullData::Ptr object) {
  if (object) {
    object->SetHueMean(CalculatePlaneMean(object, ObjectBasicData::HUE_PLANE));
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::SatMeanFeature(ObjectFullData::Ptr object) {
  if (object) {
    object->SetSatMean(CalculatePlaneMean(object,
                                          ObjectBasicData::SATURATION_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::IntensityMeanFeature(std::shared_ptr<
    ObjectFullData> object) {
  if (object) {
    object->SetIntensityMean(CalculatePlaneMean(object,
                                                 ObjectBasicData::INTENSITY_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::RedMeanFeature(ObjectFullData::Ptr object) {
  if (object) {
    object->SetRedMean(CalculatePlaneMean(object, ObjectBasicData::RED_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::GreenMeanFeature(std::shared_ptr<
    ObjectFullData> object) {
  if (object) {
    object->SetGreenMean(CalculatePlaneMean(object,
                                            ObjectBasicData::GREEN_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::BlueMeanFeature(ObjectFullData::Ptr object) {
  if (object) {
    object->SetBlueMean(CalculatePlaneMean(object,
                                           ObjectBasicData::BLUE_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::GrayMeanFeature(ObjectFullData::Ptr object) {
  if (object) {
    object->SetGrayMean(CalculatePlaneMean(object,
                                           ObjectBasicData::GRAY_PLANE));
  }
}
#endif  //_FEATURE_FACTORY_H_
