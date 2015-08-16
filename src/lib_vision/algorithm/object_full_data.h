/**
 * \file	object_full_data.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_FULL_OBJECT_DATA_H_
#define VISION_FILTER_FULL_OBJECT_DATA_H_

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <lib_vision/algorithm/object_basic_data.h>
#include <lib_vision/algorithm/object_tracking_data.h>
#include <lib_vision/algorithm/object_ranking_data.h>
#include <lib_vision/algorithm/feature_vec.h>

// Simple container class that is created with the contour.
// It inherits from the different caracteristic of an object
// (in the time domain, as a contour and compared to others)
// It is important to note that it does not calculate all the
// basic characteristic, it waits until it is ask from the object
// to calculated. That way, we do not waste calculation time for information
// we wont use.
// Also, for tracking and ranking data, it is necessary
// to fill the object with the help of ObjectRanker and FrameMemory.
class ObjectFullData : public OBjectTrackingData,
                       public ObjectBasicData,
                       public ObjectRankingData,
                       public FeatureVec {
 public:
  typedef std::vector<std::shared_ptr<ObjectFullData>> FullObjectPtrVec;

  ObjectFullData(const cv::Mat &originalImage, const cv::Mat &binaryImage,
                 const contour_t &contour);

  virtual ~ObjectFullData(){};
};

#endif
