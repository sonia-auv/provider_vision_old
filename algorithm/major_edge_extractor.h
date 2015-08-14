/**
 * \file	major_edge_extractor.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_MAJOR_EDGE_EXTRACTOR_H_
#define VISION_FILTER_MAJOR_EDGE_EXTRACTOR_H_

#include <opencv2/opencv.hpp>

//=============================================================================
//		CONTAINER CLASS
//-----------------------------------------------------------------------------
//
class ReferencePoint {
 public:
  ReferencePoint(float pix_val, int max_val_index);

  float _pix_value;
  int _reference_max_index;
};

typedef ReferencePoint *RefPointPtr;
typedef cv::Mat_<RefPointPtr> RefImage;

//-----------------------------------------------------------------------------
//
class RefKernel {
 public:
  RefKernel(const RefPointPtr &north, const RefPointPtr &west,
            const RefPointPtr &center);

  ~RefKernel(){};
  RefPointPtr _north;
  RefPointPtr _west;
  RefPointPtr _center;
};

//=============================================================================
//		MAIN CLASS
//-----------------------------------------------------------------------------
//
class MajorEdgeExtractor {
 public:
  cv::Mat ExtractEdge(const cv::Mat &image, int extreme_minimum);

 private:
  static const float PERCENT_OF_VAL_FOR_VALUE_CONNECTION;

  void Init(const cv::Size &size);

  void Clean();

  void CreateRefImage(const cv::Size &size);

  void AddRef(int x, int y, float value);

  bool IsAloneRef(const RefKernel &ref_kernel) const;

  bool IsNorthExist(const RefKernel &ref_kernel) const;

  bool IsWestExist(const RefKernel &ref_kernel) const;

  bool IsBothNortAndWestExist(const RefKernel &ref_kernel) const;

  bool IsValueConnected(const RefPointPtr ref, float value) const;

  bool IsValueGreater(const RefPointPtr ref, float value) const;

  bool IsJunction(const RefKernel &ref_kernel, float value) const;

  void SetLink(const RefPointPtr ref, float value, int x, int y);

  void SetJunction(RefKernel &ref_kernel, float value, int x, int y);

  float GetValInReferenceVec(int index);

  float GetValInReferenceVec(RefPointPtr ptr);

  void SetValInReferenceVec(int index, float value);

  void SetValInReferenceVec(RefPointPtr ptr, float value);

  RefImage _ref_image;
  std::vector<float> _max_value_reference;
  cv::Size _img_size;

  friend class MajorEdgeExtractorUT;
};

//=============================================================================
// 	INLINE CODE FUNCTION
//-----------------------------------------------------------------------------
//
inline void MajorEdgeExtractor::CreateRefImage(const cv::Size &size) {
  _ref_image = RefImage(size);
  // Free the RefPoint allocated previously.
  for (int y = 0, rows = _ref_image.rows, cols = _ref_image.cols; y < rows;
       y++) {
    RefPointPtr *ptr = _ref_image.ptr<RefPointPtr>(y);
    for (int x = 0; x < cols; x++) {
      ptr[x] = nullptr;
    }
  }
}

//-----------------------------------------------------------------------------
//
inline void MajorEdgeExtractor::AddRef(int x, int y, float value) {
  _max_value_reference.push_back(value);
  _ref_image.at<RefPointPtr>(y, x) =
      new ReferencePoint(value, _max_value_reference.size() - 1);
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsAloneRef(const RefKernel &ref_kernel) const {
  // if north or west exist, not alone
  return !(IsNorthExist(ref_kernel) || IsWestExist(ref_kernel));
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsNorthExist(
    const RefKernel &ref_kernel) const {
  return ref_kernel._north != nullptr;
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsWestExist(const RefKernel &ref_kernel) const {
  return ref_kernel._west != nullptr;
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsBothNortAndWestExist(
    const RefKernel &ref_kernel) const {
  return IsNorthExist(ref_kernel) && IsWestExist(ref_kernel);
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsValueConnected(const RefPointPtr ref,
                                                 float value) const {
  if (ref != nullptr)
    return float(ref->_pix_value) * PERCENT_OF_VAL_FOR_VALUE_CONNECTION <=
           value;
  return false;
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsValueGreater(const RefPointPtr ref,
                                               float value) const {
  if (ref != nullptr)
    return _max_value_reference[ref->_reference_max_index] < value;
  return false;
}

//-----------------------------------------------------------------------------
//
inline bool MajorEdgeExtractor::IsJunction(const RefKernel &ref_kernel,
                                           float value) const {
  return IsValueConnected(ref_kernel._north, value) &&
         IsValueConnected(ref_kernel._west, value);
}

//-----------------------------------------------------------------------------
//

inline void MajorEdgeExtractor::SetLink(const RefPointPtr ref, float value,
                                        int x, int y) {
  if (ref != nullptr) {
    _ref_image.at<RefPointPtr>(y, x) =
        new ReferencePoint(value, ref->_reference_max_index);
    if (IsValueGreater(ref, value)) {
      SetValInReferenceVec(ref, value);
    }
  }
}

//-----------------------------------------------------------------------------
//
inline float MajorEdgeExtractor::GetValInReferenceVec(int index) {
  return _max_value_reference[index];
}

//-----------------------------------------------------------------------------
//
inline float MajorEdgeExtractor::GetValInReferenceVec(RefPointPtr ptr) {
  return GetValInReferenceVec(ptr->_reference_max_index);
}

//-----------------------------------------------------------------------------
//
inline void MajorEdgeExtractor::SetValInReferenceVec(int index, float value) {
  _max_value_reference[index] = value;
}

//-----------------------------------------------------------------------------
//
inline void MajorEdgeExtractor::SetValInReferenceVec(RefPointPtr ptr,
                                                     float value) {
  SetValInReferenceVec(ptr->_reference_max_index, value);
}

#endif  //_MAJOR_EDGE_EXTRACTOR_H_
