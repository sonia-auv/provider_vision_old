/**
 * \file	double_parameter.h
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

#ifndef LIB_VISION_PARAMETERS_DOUBLE_PARAMETER_H_
#define LIB_VISION_PARAMETERS_DOUBLE_PARAMETER_H_

#include <memory>
#include <lib_vision/parameter.h>

namespace lib_vision {

class DoubleParameter : public Parameter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DoubleParameter>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit DoubleParameter(const std::string &_name, const double _value,
                           const double _min, const double _max,
                           std::vector<Parameter::Ptr> &param_vector,
                           const std::string &description = "")
      : Parameter(_name, DOUBLE, description),
        value(_value),
        max(_max),
        min(_min) {
    param_vector.push_back(Parameter::Ptr(dynamic_cast<Parameter *>(this)));
  }

  ~DoubleParameter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  // Boolean, no == or != since double, with precision, it is not a good
  // practice.
  template <class TYPE>
  bool operator>(const TYPE _value) {
    return double(_value) > value;
  }

  template <class TYPE>
  bool operator<(const TYPE _value) {
    return _value < value;
  }

  // Aritmethic
  template <class TYPE>
  void operator+=(const TYPE _value) {
    value += double(_value);
  }

  template <class TYPE>
  void operator-=(const TYPE _value) {
    value -= double(_value);
  }

  template <class TYPE>
  void operator*=(const TYPE _value) {
    value *= double(_value);
  }

  template <class TYPE>
  void operator/=(const TYPE _value) {
    value /= double(_value);
  }

  template <class TYPE>
  double operator+(const TYPE _value) {
    return value + double(_value);
  }

  template <class TYPE>
  double operator-(const TYPE _value) {
    return value - double(_value);
  }

  template <class TYPE>
  double operator*(const TYPE _value) {
    return value * double(_value);
  }

  template <class TYPE>
  double operator/(const TYPE _value) {
    return value / double(_value);
  }

  // We have two set of seter/getter. The operator's one are for the filters
  // so the params are easily usable.
  // Set
  template <class TYPE>
  void operator=(const TYPE _value) {
    setValue(_value);
  }

  template <class TYPE>
  inline void setValue(const TYPE _value) {
    int x = static_cast<int>(_value);
    value = x < min ? min : (x > max ? max : x);
  }

  // Get
  // Here we put priority on operator() since it
  // is going to be called more often.
  double operator()() const { return value; }

  inline double getValue() const { return operator()(); }

  virtual inline std::string GetStringValue() const override {
    return std::to_string(value);
  }

  virtual std::string toString() const {
    std::stringstream ss;
    ss << getName() << SEPARATOR << "Double" << SEPARATOR << value << SEPARATOR
       << min << SEPARATOR << max << SEPARATOR << getDescription() << ";";
    return ss.str();
  }

 private:
  double value;

  double max;

  double min;
};

}  // namespace lib_vision

#endif  // LIB_VISION_PARAMETERS_DOUBLE_PARAMETER_H_
