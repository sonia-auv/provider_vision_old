/**
 * \file	time.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */
#include <lib_vision/algorithm/performance_evaluator.h>

//=================================================================
//  CONSTRUCTOR
PerformanceEvaluator::PerformanceEvaluator()
: _tick_frequency(cv::getTickFrequency()),
  _start_tick_count(cv::getTickCount())
{
  // Should never occur but...
  if ( _tick_frequency == 0.0f)
  {
    _tick_frequency = 1;
  }
}

//=================================================================
// METHOD
