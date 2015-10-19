/**
 * \file	time.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_PERFORMANCE_EVALUATOR_H_
#define VISION_FILTER_PERFORMANCE_EVALUATOR_H_

#include "opencv2/opencv.hpp"

/*
 * Class to easily calculate process time
 * At construction, it takes the current tick count
 * and the tick frequency.
 * After that, when you call GetExecTime, it returns
 * the time in millisecond since its construction.
 * you can update the start time (time since construction)
 * by calling UpdateStartTime()
 *
 */

class PerformanceEvaluator {
 public:
  PerformanceEvaluator();
  ~PerformanceEvaluator(){};

  // Return the time in second since construction or call to UpdateStartTime
  double GetExecTimeSec();

  // Reset the time reference
  void UpdateStartTime();

 private:
  double _start_tick_count;
  double _tick_frequency;
};

//=============================================================================
//  INLINE FUNCTION

//-----------------------------------------------------------------------------
//
inline double PerformanceEvaluator::GetExecTimeSec() {
  return (cv::getTickCount() - _start_tick_count) / _tick_frequency;
}

//-----------------------------------------------------------------------------
//
inline void PerformanceEvaluator::UpdateStartTime() {
  _start_tick_count = cv::getTickCount();
}

#endif
