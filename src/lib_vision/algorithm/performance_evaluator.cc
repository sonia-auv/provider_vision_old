/**
 * \file	performance_evaluator.cc
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

#include <lib_vision/algorithm/performance_evaluator.h>

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
PerformanceEvaluator::PerformanceEvaluator()
    : _tick_frequency(cv::getTickFrequency()),
      _start_tick_count(cv::getTickCount()) {
  // Should never occur but...
  if (_tick_frequency == 0.0f) {
    _tick_frequency = 1;
  }
}
