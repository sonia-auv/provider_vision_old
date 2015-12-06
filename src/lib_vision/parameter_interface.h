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

#ifndef LIB_VISION_PARAMETER_INTERFACE_H_
#define LIB_VISION_PARAMETER_INTERFACE_H_

#include <memory>
#include <string>
#include <cstdlib>
#include <string>
#include <sstream>
#include <lib_atlas/macros.h>

namespace lib_vision {

class ParameterInterface {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ParameterInterface>;

  //============================================================================
  // P U B L I C   C / D T O R S

  ParameterInterface() = default;

  virtual ~ParameterInterface() ATLAS_NOEXCEPT = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void SetDescription(const std::string &description) = 0;

  virtual std::string GetDescription() const = 0;

  virtual void SetName(const std::string &name) = 0;

  virtual std::string GetName() const = 0;

  virtual std::string ToString() const = 0;

  virtual std::string GetType() const = 0;

  virtual std::string GetStringValue() const = 0;

  virtual void SetStringValue(const std::string &) = 0;
};

}  // namespace lib_vision

#endif  // LIB_VISION_PARAMETER_INTERFACE_H_
