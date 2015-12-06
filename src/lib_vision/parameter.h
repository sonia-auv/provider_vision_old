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

#ifndef LIB_VISION_FILTER_PARAMETER_H_
#define LIB_VISION_FILTER_PARAMETER_H_

#include <memory>
#include <string>
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>
#include <lib_atlas/macros.h>

namespace lib_vision {

template <typename Tp_>
class Parameter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Parameter<Tp_>>;

  static const char SEPARATOR = '|';

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Parameter(const std::string &name, const std::string &description,
                     std::vector<Parameter *> *vector = nullptr);

  virtual ~Parameter() ATLAS_NOEXCEPT = default;

  //============================================================================
  // P U B L I C   O P E R A T O R S

  template <class Ut_>
  bool operator>(const Ut_ &rhs) {
    return value_ > rhs;
  }

  template <class Ut_>
  bool operator<(const Ut_ &rhs) {
    return value_ < rhs;
  }

  template <class Ut_>
  bool operator==(const Ut_ &rhs) {
    return rhs == value_;
  }

  template <class Ut_>
  bool operator!=(const Ut_ &rhs) {
    return rhs != value_;
  }

  template <class Ut_>
  void operator+=(const Ut_ &rhs) {
    value_ += rhs;
  }

  template <class Ut_>
  void operator++() {
    value_++;
  }

  template <class Ut_>
  void operator-=(const Ut_ &rhs) {
    value_ -= rhs;
  }

  template <class Ut_>
  void operator--() {
    value_--;
  }

  template <class Ut_>
  void operator*=(const Ut_ &rhs) {
    value_ *= rhs;
  }

  template <class Ut_>
  void operator/=(const Ut_ &rhs) {
    value_ /= rhs;
  }

  template <class Ut_>
  int operator+(const Ut_ &rhs) {
    return value_ + rhs;
  }

  template <class Ut_>
  int operator-(const Ut_ &rhs) {
    return value_ - rhs;
  }

  template <class Ut_>
  int operator*(const Ut_ &rhs) {
    return value_ * rhs;
  }

  template <class Ut_>
  int operator/(const Ut_ &rhs) {
    return value_ / rhs;
  }

  template <class Ut_>
  void operator=(const Ut_ &rhs) {
    SetValue(rhs);
  }

  //============================================================================
  // P U B L I C   M E T H O D S

  inline void SetDescription(const std::string &description);

  inline std::string GetDescription() const;

  inline void SetName(const std::string &name);

  inline std::string GetName() const;

  virtual std::string ToString() const;

  inline void SetValue(const Tp_ &value);

  inline const Tp_ &GetValue() const;

  inline std::string GetType() const;

  virtual std::string GetStringValue() const;

 protected:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  std::string name_;

  std::string description_;

  Tp_ value_;
};

}  // namespace lib_vision

#include <lib_vision/parameter_inl.h>

#endif  // LIB_VISION_FILTER_PARAMETER_H_
