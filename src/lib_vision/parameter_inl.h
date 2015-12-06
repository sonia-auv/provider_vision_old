/**
 * \file	filter_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
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
#error This file may only be included from parameter.h
#endif

#include <string>
#include <cstdlib>
#include <cxxabi.h>
#include <opencv/cv.h>
#include <vector>
#include <lib_atlas/macros.h>

namespace lib_vision {

namespace details {

template <typename T>
struct TypeName {
  static std::string Get() {
    int status;
    std::string tname = typeid(T).name();
    char *demangled_name =
        abi::__cxa_demangle(tname.c_str(), NULL, NULL, &status);
    if (status == 0) {
      tname = demangled_name;
      std::free(demangled_name);
    }
    return tname;
  }
};

template <>
struct TypeName<int> {
  static std::string Get() { return "Integer"; }
};

template <>
struct TypeName<bool> {
  static std::string Get() { return "Boolean"; }
};

template <>
struct TypeName<double> {
  static std::string Get() { return "Double"; }
};

template <>
struct TypeName<std::string> {
  static std::string Get() { return "String"; }
};

template <>
struct TypeName<cv::Mat> {
  static std::string Get() { return "Matrix"; }
};

}  // namespace details

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE Parameter<Tp_>::Parameter(
    const std::string &name, const std::string &description,
    std::vector<ParameterInterface *> *vector)
    : name_(name), description_(description) {
  if (vector != nullptr) {
    vector->push_back(this);
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetDescription(
    const std::string &description) {
  description_ = description;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetDescription() const {
  return description_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetName(const std::string &name) {
  name_ = name;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetName() const {
  return name_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::ToString() const {
  std::stringstream ss;
  ss << GetName() << SEPARATOR << GetType() << SEPARATOR << GetStringValue()
     << SEPARATOR << GetDescription() << ";";
  return ss.str();
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE void Parameter<Tp_>::SetValue(const Tp_ &value) {
  value_ = value;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE const Tp_ &Parameter<Tp_>::GetValue() const {
  return value_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetType() const {
  return details::TypeName<Tp_>::Get();
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_INLINE std::string Parameter<Tp_>::GetStringValue() const {
  return std::to_string(value_);
}

}  // namespace lib_vision
