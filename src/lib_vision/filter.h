/**
 * \file	filter.h
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


#ifndef VISION_FILTER_FILTER_H_
#define VISION_FILTER_FILTER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <lib_vision/parameters/integer_parameter.h>
#include <lib_vision/parameters/boolean_parameter.h>
#include <lib_vision/parameters/double_parameter.h>
#include <lib_vision/parameters/string_parameter.h>
#include <lib_vision/parameter.h>
#include <lib_vision/global_param_handler.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

class Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  // KEEPING A REFERENCE TO GlobalParamHandler. VERY IMPORTANT
  explicit Filter(const GlobalParamHandler &globalParams)
      : global_params_(const_cast<GlobalParamHandler &>(globalParams)),
        //enable_("Enable", false, parameters_),
        // Explicit construction not needed here... Just reminder it exist.
        parameters_() {}

  virtual ~Filter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  // Filter MAIN PROCESS
  virtual void execute(cv::Mat &image) = 0;

  // Filter API
  // Normally no process, but could want to load a file etc.
  virtual void destroy() {}

  virtual void init() {}

  // Filter param getter/setter
  // Cannot return reference as they are constructed
  // at each call.
  std::string getParamList() {
    std::stringstream ss;
    for (int i = 0; i < int(parameters_.size()); i++) {
      // Should never append since the vector's object
      // are added by the said object which are member of
      // the filter class...
      if (parameters_[i] != NULL) {
        ss << parameters_[i]->toString();
      }
    }
    return ss.str();
  }

  const std::vector<Parameter *> &GetParameters() const { return parameters_; }

  std::string getParamValue(const std::string &name) {
    std::string returnString("");
    for (int i = 0; i < int(parameters_.size()); i++) {
      // Here we give it a local value to limit the
      // access to the vector (optimisation)
      Parameter *param = parameters_[i];

      // NULL element should never append since the vector's object
      // are added by the said object (which cannot be null if
      // it gets to the constructor) and which are member of
      // the filter class (which mean they are alive as long as the
      // filter is alive)...
      if (param != NULL) {
        // Is it the param we are searching
        if (param->getName() == name) {
          returnString = param->toString();
        }
      }
    }
    return returnString;
  }

  void setParamValue(const std::string &name, std::string value) {
    for (int i = 0; i < int(parameters_.size()); i++) {
      // Here we give it a local value to limit the
      // access to the vector (optimisation)
      Parameter *param = parameters_[i];

      // NULL element should never append since the vector's object
      // are added by the said object (which cannot be null if
      // it gets to the constructor) and which are member of
      // the filter class (which mean they are alive as long as the
      // filter is alive)...
      if (param != NULL) {
        // Is it the param we are searching
        if (param->getName() == name) {
          // Need to dynamic cast in order to have
          // access to the function of the specific params type.
          // Necessary to instanciate them here, because of
          // error:   crosses initialization of
          // see:
          // http://stackoverflow.com/questions/11578936/getting-a-bunch-of-crosses-initialization-error
          // for more info.
          BooleanParameter *p_bool = NULL;
          IntegerParameter *p_int = NULL;
          DoubleParameter *p_double = NULL;
          StringParameter *p_str = NULL;
          switch (param->getType()) {
            case Parameter::BOOL:
              p_bool = dynamic_cast<BooleanParameter *>(param);
              // Just in case the cast didn't work.
              if (p_bool == NULL) {
                break;
              }
              p_bool->setValue(BooleanParameter::FromStringToBool(value));
              break;
            case Parameter::INTEGER:
              p_int = dynamic_cast<IntegerParameter *>(param);
              // Just in case the cast didn't work.
              if (p_int == NULL) {
                break;
              }
              p_int->setValue(atoi(value.c_str()));
              break;
            case Parameter::DOUBLE:
              p_double = dynamic_cast<DoubleParameter *>(param);
              // Just in case the cast didn't work.
              if (p_double == NULL) {
                break;
              }
              p_double->setValue(atof(value.c_str()));
              break;
            case Parameter::STRING:
              p_str = dynamic_cast<StringParameter *>(param);
              // Just in case the cast didn't work.
              if (p_str == NULL) {
                break;
              }
              p_str->setValue(value);
              break;
            // Nothing to default.
            default:
              break;
          }
        }
      }
    }
  }

  // Name of the filter handlers
  inline const std::string getName() { return name_; }

  inline void setName(const std::string &name) { name_ = name; }

  // Wrapper for a call to _globalParms
  // NotifyString, to be put on the result topic
  inline void notify_str(const std::string &_notifyString) {
    global_params_.setNotifyString(_notifyString);
  }

  inline const std::string get_notify_str() {
    return global_params_.getNotifyString();
  }

  // Global parameters SeaGoatAPI
  // Creator.
  void global_param_int(const std::string &name, const int value, const int min,
                        const int max) {
    global_params_.addParam(
        new IntegerParameter(name, value, max, min, parameters_));
  }

  void global_param_double(const std::string &name, const double value,
                           const double min, const double max) {
    global_params_.addParam(
        new DoubleParameter(name, value, max, min, parameters_));
  }

  void global_param_bool(const std::string &name, const bool value) {
    global_params_.addParam(new BooleanParameter(name, value, parameters_));
  }

  void global_param_string(const std::string &name, const std::string &value) {
    global_params_.addParam(new StringParameter(name, value, parameters_));
  }

  void global_param_mat(const std::string &name, cv::Mat &mat) {
    global_params_.addParam(new MatrixParameter(name, mat, parameters_));
  }

 protected:
  // KEEPING A REFERENCE. VERY IMPORTANT
  GlobalParamHandler &global_params_;
//  BooleanParameter enable_;

  // vector parameter, so we can list them.
  std::vector<Parameter *> parameters_;

  // Useful to identify the filter.
  std::string name_;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_FILTER_H_
