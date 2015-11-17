/**
 * \file	rotate.h
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


#ifndef VISION_FILTER_ROTATE_H_
#define VISION_FILTER_ROTATE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

class Rotate : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Rotate(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("enable", false, parameters_),
        _rotate_type("Rotation_type", 0, 0, 3, parameters_,
                     "Rotate type: 0=NONE, 1=x axis, 2=y axis, 3=all axis") {
    setName("Rotate");
  }

  virtual ~Rotate() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      switch (_rotate_type()) {
        case 0:
          break;
        case 1:
          cv::flip(image, image, 0);
          break;
        case 2:
          cv::flip(image, image, 1);
          break;
        case 3:
          cv::flip(image, image, -1);
          break;
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _rotate_type;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_ROTATE_H_
