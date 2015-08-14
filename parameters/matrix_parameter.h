/**
 * \file	Matrix.h
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	11/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_MATRIX_H_
#define VISION_FILTER_MATRIX_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/parameter.h>
#include <opencv/cv.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

// Matrix are more of global param, cannot be set via UI, so does not give
// description nor param vector
class MatrixParameter : public Parameter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit MatrixParameter(const std::string &name, cv::Mat &mat,
                           std::vector<Parameter *> &param_vector)
      : Parameter(name, MATRIX, ""), value(mat) {}

  ~MatrixParameter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  void operator=(const cv::Mat &_value) { value = _value; }

  inline void setValue(const cv::Mat &_value) { value = _value; }

  inline cv::Mat getValue() const { return value; }

  virtual std::string GetStringValue() const override { return ""; }

  virtual std::string toString() const override {
    std::stringstream ss;
    ss << "Matrix" << SEPARATOR << getName() << SEPARATOR << value.size
       << SEPARATOR << getDescription() << ";";
    return ss.str();
  }

 private:
  cv::Mat value;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_MATRIX_H_
