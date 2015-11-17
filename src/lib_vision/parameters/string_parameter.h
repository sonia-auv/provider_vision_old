/**
 * \file	string_parameter.h
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

#ifndef LIB_VISION_PARAMETERS_STRING_PARAMETER_H_
#define LIB_VISION_PARAMETERS_STRING_PARAMETER_H_

#include <lib_vision/parameter.h>

namespace lib_vision {

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

#endif  // LIB_VISION_PARAMETERS_STRING_PARAMETER_H_
