/**
 * \file	TestFilter.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_TEST_FILTER_H_
#define VISION_FILTER_TEST_FILTER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class TestFilter : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit TestFilter(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_, "Enable the filter"),
        _int("Test_int", 2, 1, 3, parameters_, "Test int"),
        _bool("test_bool", false, parameters_),
        _str("Test_string", "teststring", parameters_),
        _double("test_double", 2.0f, 1, 3, parameters_) {
    setName("TestFilter");
  }

  virtual ~TestFilter() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void init() {
    // Testing creation of global param
    global_param_int("test_int", 3, 1, 10);
    global_param_bool("test_bool", false);
    global_param_double("test_double", 2.0f, 0.0f, 15.0f);
    global_param_string("test_string", "string");
    // global_param_mat("testmat", cv::Mat::zeros(100,100,CV_8UC1));
  }

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::Mat imageOriginal = global_params_.getOriginalImage();
      imageOriginal = cv::Mat::zeros(1, 1, CV_8UC1);
      IntegerParameter *int_test = dynamic_cast<IntegerParameter *>(
          this->global_params_.getParam("test_int"));
      if (int_test != NULL) notify_str("IntegerParameter OK");

      BooleanParameter *bool_test = dynamic_cast<BooleanParameter *>(
          this->global_params_.getParam("test_bool"));
      if (bool_test != NULL) notify_str("Bool OK");

      DoubleParameter *double_test = dynamic_cast<DoubleParameter *>(
          this->global_params_.getParam("test double"));
      if (double_test != NULL) notify_str("DoubleParameter OK");

      StringParameter *string_test = dynamic_cast<StringParameter *>(
          this->global_params_.getParam("test_string"));
      if (string_test != NULL) notify_str("String OK");
      image = global_params_.getOriginalImage();
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _int;
  BooleanParameter _bool;
  StringParameter _str;
  DoubleParameter _double;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_TEST_FILTER_H_
