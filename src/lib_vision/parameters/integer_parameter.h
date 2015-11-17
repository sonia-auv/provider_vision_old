/**
 * \file	Integer.h
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	11/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_INTEGER_H_
#define VISION_FILTER_INTEGER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/parameter.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

class IntegerParameter : public Parameter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit IntegerParameter(const std::string &_name, const int _value,
                            const int _min, const int _max,
                            std::vector<Parameter *> &param_vector,
                            const std::string &description = "")
      : Parameter(_name, INTEGER, description),
        value(_value),
        min(_min),
        max(_max) {
    param_vector.push_back(this);
  }

  virtual ~IntegerParameter() {}

  //============================================================================
  // P U B L I C   M E T H O D S
  // Boolean
  template <class TYPE>
  bool operator>(const TYPE _value) {
    return double(_value) > value;
  }

  template <class TYPE>
  bool operator<(const TYPE _value) {
    return double(_value) < value;
  }

  template <class TYPE>
  bool operator==(const TYPE _value) {
    return double(_value) == value;
  }

  template <class TYPE>
  bool operator!=(const TYPE _value) {
    return double(_value) != value;
  }

  // Aritmethic
  template <class TYPE>
  void operator+=(const TYPE _value) {
    value += double(_value);
  }

  template <class TYPE>
  void operator++() {
    value++;
  }

  template <class TYPE>
  void operator-=(const TYPE _value) {
    value -= double(_value);
  }

  template <class TYPE>
  void operator--() {
    value--;
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
  int operator+(const TYPE _value) {
    return value + double(_value);
  }

  template <class TYPE>
  int operator-(const TYPE _value) {
    return value - double(_value);
  }

  template <class TYPE>
  int operator*(const TYPE _value) {
    return value * double(_value);
  }

  template <class TYPE>
  int operator/(const TYPE _value) {
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
    // Clamping the passed value in the min an max.
    int x = static_cast<int>(_value);
    value = x < min ? min : (x > max ? max : x);
  }

  // Get
  inline int operator()() const { return getValue(); }

  inline int getValue() const { return value; }

  virtual inline std::string GetStringValue() const override {
    return std::to_string(value);
  }

  virtual std::string toString() const override {
    std::stringstream ss;
    ss << getName() << SEPARATOR << "Integer" << SEPARATOR << value << SEPARATOR
       << min << SEPARATOR << max << SEPARATOR << getDescription() << ";";
    return ss.str();
  }

 private:
  int value;
  int min;
  int max;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_INTEGER_H_
