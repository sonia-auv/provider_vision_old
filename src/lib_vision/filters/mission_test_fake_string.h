/**
 * \file	mission_test_fake_string.h
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

#ifndef LIB_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
#define LIB_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class MissionTestFakeString : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MissionTestFakeString>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit MissionTestFakeString(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _string("String_to_return", "test", &parameters_) {
    SetName("MissionTestFakeString");
  }

  virtual ~MissionTestFakeString() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (_enable()) {
      NotifyString(_string());
    }
  }

 private:
  // Params
  Parameter<bool> _enable;
  Parameter<std::string> _string;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_MISSION_TEST_FAKE_STRING_H_
