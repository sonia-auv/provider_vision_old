/**
 * \file	test_filter.h
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

#ifndef LIB_VISION_FILTERS_TEST_FILTER_H_
#define LIB_VISION_FILTERS_TEST_FILTER_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class TestFilter : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TestFilter>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TestFilter(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_, "Enable the filter"),
        _int("Test_int", 2, 1, 3, &parameters_, "Test int"),
        _bool("test_bool", false, &parameters_),
        _str("Test_string", "teststring", &parameters_),
        _double("test_double", 2.0f, 1, 3, &parameters_) {
    SetName("TestFilter");
  }

  virtual ~TestFilter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void init() {
    // Testing creation of global param
    GlobalParamInteger("test_int", 3, 1, 10);
    GlobalParamBoolean("test_bool", false);
    GlobalParamDouble("test_double", 2.0f, 0.0f, 15.0f);
    GlobalParamString("test_string", "string");
    // global_param_mat("testmat", cv::Mat::zeros(100,100,CV_8UC1));
  }

  virtual void Execute(cv::Mat &image) {
    if (_enable()) {
      cv::Mat imageOriginal = global_params_.getOriginalImage();
      imageOriginal = cv::Mat::zeros(1, 1, CV_8UC1);
      RangedParameter<int> *int_test(dynamic_cast<RangedParameter<int> *>(
          global_params_.getParam("test_int")));
      if (int_test != nullptr) {
        NotifyString("RangedParameter<int> OK");
      }

      Parameter<bool> *bool_test(dynamic_cast<Parameter<bool> *>(
          global_params_.getParam("test_bool")));
      if (bool_test != nullptr) {
        NotifyString("Bool OK");
      }

      RangedParameter<double> *double_test(
          dynamic_cast<RangedParameter<double> *>(
              global_params_.getParam("test double")));
      if (double_test != nullptr) {
        NotifyString("RangedParameter<double> OK");
      }

      Parameter<std::string> *string_test(
          dynamic_cast<Parameter<std::string> *>(
              global_params_.getParam("test_string")));
      if (string_test != nullptr) {
        NotifyString("String OK");
      }
      image = global_params_.getOriginalImage();
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> _enable;
  RangedParameter<int> _int;
  Parameter<bool> _bool;
  Parameter<std::string> _str;
  RangedParameter<double> _double;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_TEST_FILTER_H_
