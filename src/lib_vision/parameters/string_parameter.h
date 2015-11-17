/**
 * \file	String.h
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	11/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

// We are not respecting our coding style here just to be sure we
// dont redefine some macro of std::string.
#ifndef VISION_FILTER_STRINGPARAM_H_
#define VISION_FILTER_STRINGPARAM_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/parameter.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

class StringParameter : public Parameter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit StringParameter(std::string _name, std::string _value,
                           std::vector<Parameter *> &param_vector,
                           const std::string &description = "")
      : Parameter(_name, STRING, description), value(_value) {
    param_vector.push_back(this);
  }

  virtual ~StringParameter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  // Setter
  void operator=(const std::string &_value) { setValue(_value); }

  inline void setValue(const std::string &_value) { value = _value; }

  // Getter
  inline std::string getValue() const { return value; }

  std::string operator()() const { return value; }

  virtual std::string toString() const override {
    std::stringstream ss;
    ss << getName() << SEPARATOR << "String" << SEPARATOR << value
       << SEPARATOR /*min*/
       << SEPARATOR /*max*/
       << SEPARATOR << getDescription() << ";";
    return ss.str();
  }

  virtual inline std::string GetStringValue() const override { return value; }

 private:
  std::string value;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_STRINGPARAM_H_
