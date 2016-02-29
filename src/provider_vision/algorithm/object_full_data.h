/**
 * \file	object_full_data.h
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

#ifndef PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_
#define PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <provider_vision/algorithm/object_basic_data.h>
#include <provider_vision/algorithm/object_tracking_data.h>
#include <provider_vision/algorithm/object_ranking_data.h>
#include "provider_vision/algorithm/object_feature.h"

namespace provider_vision {

// Simple container class that is created with the contour.
// It inherits from the different caracteristic of an object
// (in the time domain, as a contour and compared to others)
// It is important to note that it does not calculate all the
// basic characteristic, it waits until it is ask from the object
// to calculated. That way, we do not waste calculation time for information
// we wont use.
// Also, for tracking and ranking data, it is necessary
// to fill the object with the help of ObjectRanker and FrameMemory.
class ObjectFullData : public ObjectTrackingData,
                       public ObjectBasicData,
                       public ObjectRankingData,
                       public ObjectFeatureData {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectFullData>;

  typedef std::vector<ObjectFullData::Ptr> FullObjectPtrVec;

  //============================================================================
  // P U B L I C   C / D T O R S

  ObjectFullData(const cv::Mat &originalImage, const cv::Mat &binaryImage,
                 const Contour &contour);

  virtual ~ObjectFullData(){};
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool AreaSorts(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
  return a->GetArea() < b->GetArea();
}

//------------------------------------------------------------------------------
//
inline bool RatioSorts(ObjectFullData::Ptr a, ObjectFullData::Ptr b) {
  return a->GetRatio() < b->GetRatio();
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_ALGORITHM_OBJECT_FULL_DATA_H_