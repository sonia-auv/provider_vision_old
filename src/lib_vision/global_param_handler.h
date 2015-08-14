/**
 * \file	GlobalParamHandler.h
 * \author  Karl Ritchie <ritchie.karl@gmail.com>
 * \date	1/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_GLOBAL_PARAMETER_H_
#define VISION_FILTER_GLOBAL_PARAMETER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include <lib_vision/parameter.h>
#include <lib_vision/parameters/integer_parameter.h>
#include <lib_vision/parameters/double_parameter.h>
#include <lib_vision/parameters/string_parameter.h>
#include <lib_vision/parameters/boolean_parameter.h>
#include <lib_vision/parameters/matrix_parameter.h>
#include <sstream>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class GlobalParamHandler {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit GlobalParamHandler()
      : _notify_string(std::string()), _original_image(), _params_vec() {}

  // Since we erase everything, it is easier to delete objet first
  // then calling clear method, since erase invalidate pointer AND
  // needs an iterator. When erasing, the vector replace object so
  // that they are consecutive in memory... on long vector it
  // is "very" long to do. To prevent that, we can use reverse
  // iterator, but erase does not take it...
  ~GlobalParamHandler() {
    // Using index base for performance.
    auto size = _params_vec.size();
    for (int i = 0; i < size; i++) {
      // Delete the object
      delete (_params_vec[i]);
    }
    _params_vec.clear();
  }

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
  inline void addParam(Parameter *param) { _params_vec.push_back(param); }

  void removeParam(const std::string &name) {
    // Using iterator as it is simpler to erase.
    std::vector<Parameter *>::iterator index = _params_vec.begin();
    std::vector<Parameter *>::const_iterator end = _params_vec.end();
    bool deleted = false;
    for (; index != end && !deleted; index++) {
      if ((*index)->getName() == name) {
        _params_vec.erase(index);
        deleted = true;
      }
    }
  }

  inline Parameter *getParam(const std::string &name) const {
    // Using [] accessor for optimisation.
    for (int i = 0, size = _params_vec.size(); i < size; i++) {
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
  std::vector<Parameter *> _params_vec;
  cv::Mat _original_image;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_GLOBAL_PARAMETER_H_
