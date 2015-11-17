/**
 * \file	object_feature_factory.h
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

  // feature funcitions
  void RatioFeature(ObjectFullData::Ptr object);
  void ConvexityFeature(ObjectFullData::Ptr object);
  void PercentFilledFeature(ObjectFullData::Ptr object);
  void CircularityFeature(ObjectFullData::Ptr object);
  void PresenceConsistencyFeature(ObjectFullData::Ptr object);

  void HueMeanFeature(ObjectFullData::Ptr object);
  void SatMeanFeature(ObjectFullData::Ptr object);
  void IntensityMeanFeature(ObjectFullData::Ptr object);
  void RedMeanFeature(ObjectFullData::Ptr object);
  void GreenMeanFeature(ObjectFullData::Ptr object);
  void BlueMeanFeature(ObjectFullData::Ptr object);
  void GrayMeanFeature(ObjectFullData::Ptr object);
 private:
  float CalculatePlaneMean
      (ObjectFullData::Ptr object, int plane);

  // define the type "Pointer to method to calculate a feature"
  //typedef float (ObjectFeatureFactory::*FeatureFunction)(std::shared_ptr<ObjectFullData>);

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
    fct.second(object);//(this->*(fct.second));
    //*tmp(object);
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
  if ((object.get() != nullptr) && (object->GetRatio() == -1.0f)) {
    RotRect rect = object->GetRotatedRect();
    object->SetRatio(rect.size.width / rect.size.height);
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::ConvexityFeature(
    ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetConvexity() == -1.0f)) {
    // safety, should not happen
    float convexHull = object->GetConvexHullArea();
    float area = object->GetArea();
    if (convexHull > 0 && area > 0)
      object->SetConvexity(
          1.0f- (area / convexHull));
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::CircularityFeature(
    ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetCircularity() == -1.0f)) {
    // Here we use pow on radius instead of sqrt on area because
    // pow is less hard computation
    float radiusCircum = pow(object->GetCircumference() / (2 * M_PI), 2);
    float radiusArea = object->GetArea() / (M_PI);
    if (radiusCircum != 0 && radiusArea != 0) {
      object->SetCircularity(
          radiusCircum > radiusArea ? radiusArea / radiusCircum
                                    : radiusCircum / radiusArea);
    }
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::PresenceConsistencyFeature(
    ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetPresenceConsistency() == -1.0f)) {
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
  if ((object.get() != nullptr) && (object->GetHueMean() == -1.0f)) {
    object->SetHueMean(CalculatePlaneMean(object, ObjectBasicData::HUE_PLANE));
  }
}

//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::SatMeanFeature(ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetSatMean() == -1.0f)) {
    object->SetSatMean(CalculatePlaneMean(object,
                                          ObjectBasicData::SATURATION_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::IntensityMeanFeature(std::shared_ptr<
    ObjectFullData> object) {
  if ((object.get() != nullptr) && (object->GetIntensityMean() == -1.0f)) {
    object->SetIntensityMean(CalculatePlaneMean(object,
                                                 ObjectBasicData::INTENSITY_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::RedMeanFeature(ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetRedMean() == -1.0f)) {
    object->SetRedMean(CalculatePlaneMean(object, ObjectBasicData::RED_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::GreenMeanFeature(std::shared_ptr<
    ObjectFullData> object) {
  if ((object.get() != nullptr) && (object->GetGreenMean() == -1.0f)) {
    object->SetGreenMean(CalculatePlaneMean(object,
                                            ObjectBasicData::GREEN_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::BlueMeanFeature(ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetBlueMean() == -1.0f)) {
    object->SetBlueMean(CalculatePlaneMean(object,
                                           ObjectBasicData::BLUE_PLANE));
  }
}
//-----------------------------------------------------------------------------
//
inline void ObjectFeatureFactory::GrayMeanFeature(ObjectFullData::Ptr object) {
  if ((object.get() != nullptr) && (object->GetGrayMean() == -1.0f)) {
    object->SetGrayMean(CalculatePlaneMean(object,
                                           ObjectBasicData::GRAY_PLANE));
  }
}
#endif  //_FEATURE_FACTORY_H_
