/**
 * \file	parameter.h
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


#ifndef VISION_FILTER_PARAM_H_
#define VISION_FILTER_PARAM_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>

namespace lib_vision {

//==============================================================================
// C L A S S E S

class Parameter {
 public:
  //============================================================================
  // C O N S T A N T S   M E M B E R S

  static const char SEPARATOR = '|';

  //============================================================================
  // T Y P E D E F   A N D   E N U M

  enum TYPE { BOOL, INTEGER, DOUBLE, STRING, MATRIX, NONE };

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Parameter(const std::string &name, const TYPE type,
                     const std::string &description)
      : name(name), description(description), type(type) {}

  virtual ~Parameter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  inline void setDescription(const std::string &_description) {
    description = _description;
  }

  inline std::string getDescription() const { return description; }

  inline void setName(const std::string &_name) { name = _name; }

  inline void setType(const TYPE &_type) { type = _type; }

  inline std::string getName() const { return name; }

  inline TYPE getType() const { return type; }

  virtual std::string toString() const { return "Param,,,;"; }

  virtual std::string GetStringValue() const = 0;

 protected:
  std::string name;

  std::string description;

  TYPE type;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_PARAM_H_
