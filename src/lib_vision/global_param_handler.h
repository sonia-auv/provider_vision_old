/**
 * \file	global_param_handler.h
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

#ifndef LIB_VISION_FILTER_GLOBAL_PARAM_HANDLER_H_
#define LIB_VISION_FILTER_GLOBAL_PARAM_HANDLER_H_

#include <memory>
#include <string>
#include <sstream>
#include <lib_vision/parameter.h>
#include <lib_vision/parameters/integer_parameter.h>
#include <lib_vision/parameters/double_parameter.h>
#include <lib_vision/parameters/string_parameter.h>
#include <lib_vision/parameters/boolean_parameter.h>
#include <lib_vision/parameters/matrix_parameter.h>

namespace lib_vision {

class GlobalParamHandler {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<GlobalParamHandler>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit GlobalParamHandler()
      : _notify_string(std::string()), _params_vec(), _original_image() {}

  // Since we erase everything, it is easier to delete objet first
  // then calling clear method, since erase invalidate pointer AND
  // needs an iterator. When erasing, the vector replace object so
  // that they are consecutive in memory... on long vector it
  // is "very" long to do. To prevent that, we can use reverse
  // iterator, but erase does not take it...
  ~GlobalParamHandler() { _params_vec.clear(); }

  //============================================================================
  // P U B L I C   M E T H O D S

  // Original image handling
  // WE WANT TO RETURN A COPY, ELSE THE IMAGE WILL BE ALTERATE
  inline cv::Mat getOriginalImage() {
    // Here the return makes a copy so we are safe.
    return _original_image;
  }

  // WE WANT A COPY BECAUSE THE ORIGINAL IMAGE IS PROBABLY GOING TO BE ALTERED
  // BY THE FILTERS.
  inline void setOriginalImage(cv::Mat image) { _original_image = image; }

  // Notify string
  inline void setNotifyString(const std::string &notifyString) {
    _notify_string += notifyString;
  }

  inline const std::string getNotifyString() { return _notify_string; }

  inline const void clearNotifyString() {
    _notify_string = "";
    return;
  }

  // Params
  inline void addParam(Parameter::Ptr param) { _params_vec.push_back(param); }

  void removeParam(const std::string &name) {
    // Using iterator as it is simpler to erase.
    std::vector<Parameter::Ptr>::iterator index = _params_vec.begin();
    std::vector<Parameter::Ptr>::const_iterator end = _params_vec.end();
    bool deleted = false;
    for (; index != end && !deleted; index++) {
      if ((*index)->getName() == name) {
        _params_vec.erase(index);
        deleted = true;
      }
    }
  }

  inline Parameter::Ptr getParam(const std::string &name) const {
    // Using [] accessor for optimisation.
    for (size_t i = 0, size = _params_vec.size(); i < size; i++) {
      if (_params_vec[i]->getName() == name) {
        return _params_vec[i];
      }
    }
    return 0;
  }

  // Util
  static const char SEPARATOR = ';';

 private:
  std::string _notify_string;
  // Using pointer here to preserve the object if
  // the vector is moved in memory.
  std::vector<Parameter::Ptr> _params_vec;
  cv::Mat _original_image;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTER_GLOBAL_PARAM_HANDLER_H_
