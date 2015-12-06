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

#ifndef LIB_VISION_RANGED_PARAMETER_H_
#define LIB_VISION_RANGED_PARAMETER_H_

#include <string>
#include <vector>
#include <iostream>
#include <lib_atlas/macros.h>
#include <lib_vision/parameter.h>

namespace lib_vision {

template<typename Tp_>
class RangedParameter : public Parameter<Tp_> {
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RangedParameter<Tp_>>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit RangedParameter(const std::string &name, const std::string &description,
                     std::vector<ParameterInterface *> *vector) :
      Parameter<Tp_>(name, description, vector),
      min_(),
      max_() {}

  virtual ~RangedParameter() ATLAS_NOEXCEPT = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  const Tp_ &GetMin() const { return min_; }

  void SetMin(const Tp_ &min) { min_ = min; }

  const Tp_ &GetMax() const { return max_; }

  void SetMax(const Tp_ &max) { max_ = max; }

  std::string GetStringValue() const override {
    std::stringstream ss;
    ss << std::to_string(GetValue()) << Parameter<Tp_>::SEPARATOR;
    ss << std::to_string(GetMin()) << Parameter<Tp_>::SEPARATOR;
    ss << std::to_string(GetMax());
    return ss.str();
  }

 protected:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  Tp_ min_;

  Tp_ max_;
};

}  // namespace lib_vision

#endif  // LIB_VISION_RANGED_PARAMETER_H_
