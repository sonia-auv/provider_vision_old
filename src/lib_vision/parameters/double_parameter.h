/**
 * \file	Double.h
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	1/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_DOUBLE_H_
#define VISION_FILTER_DOUBLE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/parameter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class DoubleParameter : public Parameter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit DoubleParameter(const std::string &_name, const double _value,
                           const double _min, const double _max,
                           std::vector<Parameter *> &param_vector,
                           const std::string &description = "")
      : Parameter(_name, DOUBLE, description),
        value(_value),
        max(_max),
        min(_min) {
    param_vector.push_back(this);
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

  inline double getValue() const { return this->operator()(); }

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

}  // namespace vision_filter

#endif  // VISION_FILTER_DOUBLE_H_
