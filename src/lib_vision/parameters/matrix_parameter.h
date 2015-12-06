/**
 * \file	matrix_parameter.h
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

#ifndef LIB_VISION_PARAMETERS_MATRIX_PARAMETER_H_
#define LIB_VISION_PARAMETERS_MATRIX_PARAMETER_H_

#include <memory>
#include <opencv/cv.h>
#include <lib_vision/parameter.h>

namespace lib_vision {

// Matrix are more of global param, cannot be set via UI, so does not give
// description nor param vector
class MatrixParameter : public Parameter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MatrixParameter>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit MatrixParameter(const std::string &name, cv::Mat &mat,
                           std::vector<Parameter *> *param_vector)
      : Parameter(name, MATRIX, "", param_vector), value(mat) {}

  virtual ~MatrixParameter() {}

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

}  // namespace lib_vision

#endif  // LIB_VISION_PARAMETERS_MATRIX_PARAMETER_H_
