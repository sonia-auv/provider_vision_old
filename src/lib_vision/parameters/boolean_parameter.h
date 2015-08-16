/**
 * \file	Boolean.h
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	1/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_BOOLEAN_H_
#define VISION_FILTER_BOOLEAN_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/parameter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class BooleanParameter : public Parameter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit BooleanParameter(const std::string &_name, const bool _value,
                            std::vector<Parameter *> &param_vector,
                            const std::string &description = "")
      : Parameter(_name, BOOL, description), value(_value) {
    param_vector.push_back(this);
  }

  ~BooleanParameter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  static inline bool FromStringToBool(const std::string val) {
    // Error in refactoring, now support both cases.
    if ((val == "true") || (val == "1")) return true;
    return false;
  }

  static inline std::string FromBoolToString(const bool val) {
    if (val) return "true";
    return "false";
  }

  // Boolean operator
  bool operator==(const bool _value) { return _value == value; }

  bool operator!=(const bool _value) { return _value != value; }

  // We have two set of seter/getter. The operator's one are for the filters
  // so the params are easily usable.
  // Set
  void operator=(const bool _value) { setValue(_value); }

  inline void setValue(const bool _value) { value = _value; }

  // Get
  // Here we put priority on operator() since it
  // is going to be called more often.
  bool operator()() const { return value; }

  inline bool getValue() const { return this->operator()(); }

  virtual inline std::string GetStringValue() const override {
    return std::to_string(value);
  }

  virtual std::string toString() const override {
    std::stringstream ss;
    ss << getName() << SEPARATOR << "Boolean" << SEPARATOR
       << FromBoolToString(value) << SEPARATOR /*min*/
       << SEPARATOR                            /*max*/
       << SEPARATOR << getDescription() << ";";
    return ss.str();
  }

 private:
  bool value;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_BOOLEAN_H_
