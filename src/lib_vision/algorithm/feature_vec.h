/**
 * \file	feature_vec.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_FEATUREVEC_H_
#define VISION_FILTER_FEATUREVEC_H_

#include <lib_vision/algorithm/type_and_const.h>
#include <vector>
#include <string>
#include <stdlib.h>

class FeatureVec {
 public:
  FeatureVec(){};

  virtual ~FeatureVec(){};

  void AddFeature(float value);

  std::vector<float> GetVec();

  cv::Mat OutputVectorAsMat();

  void CreateFromMat(const cv::Mat &values);

  std::string GenerateCVine();

 private:
  std::vector<float> _features_value;
};

//=============================================================================
//	INLINE CODE FUNCTION
//-----------------------------------------------------------------------------
//
inline void FeatureVec::AddFeature(float value) {
  _features_value.push_back(value);
}

//-----------------------------------------------------------------------------
//
inline std::vector<float> FeatureVec::GetVec() { return _features_value; }

//-----------------------------------------------------------------------------
//
inline cv::Mat FeatureVec::OutputVectorAsMat() {
  cv::Mat mat(1, _features_value.size(), CV_32FC1);
  float *matPtr = mat.ptr<float>(0);
  for (int vecIter = 0, size = _features_value.size(); vecIter < size;
       vecIter++) {
    matPtr[vecIter] = _features_value[vecIter];
  }
  return mat;
}

//-----------------------------------------------------------------------------
//
inline void FeatureVec::CreateFromMat(const cv::Mat &values) {
  if (values.rows != 1 || values.cols == 0) return;

  const float *matPtr = values.ptr<float>(0);
  for (int i = 0; i < values.cols; i++) {
    _features_value.push_back(matPtr[i]);
  }
}

//-----------------------------------------------------------------------------
//
inline std::string FeatureVec::GenerateCVine() {
  std::stringstream ss;
  for (size_t i = 0; i < _features_value.size(); i++) {
    ss << _features_value[i] << ",";
  }
  return ss.str();
}

#endif
