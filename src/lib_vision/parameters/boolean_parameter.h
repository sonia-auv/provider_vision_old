/**
 * \file	boolean_parameter.h
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

#ifndef LIB_VISION_PARAMETERS_BOOLEAN_PARAMETER_H_
#define LIB_VISION_PARAMETERS_BOOLEAN_PARAMETER_H_

#include <lib_vision/parameter.h>

namespace lib_vision {

class BooleanParameter : public Parameter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BooleanParameter>;

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

}  // namespace lib_vision

#endif  // LIB_VISION_PARAMETERS_BOOLEAN_PARAMETER_H_
