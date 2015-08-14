/**
 * \file	time.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */
#include <lib_vision/algorithm/time.h>

//=================================================================
//
Time::Time() {
  this->tickFrequency = cv::getTickFrequency();
  this->startTickCount = cv::getTickCount();
}

//=================================================================
//
Time::~Time() {}

//=================================================================
//
double Time::GetExecTime() {
  return (cv::getTickCount() - this->startTickCount) / this->tickFrequency;
}

//=================================================================
//
void Time::UpdateStartTime() { this->startTickCount = cv::getTickCount(); }
