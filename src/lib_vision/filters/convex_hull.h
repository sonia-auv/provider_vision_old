/**
 * \file	convex_hull.h
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

#ifndef LIB_VISION_FILTERS_CONVEX_HULL_H_
#define LIB_VISION_FILTERS_CONVEX_HULL_H_

#include <lib_vision/filter.h>

namespace lib_vision {

class ConvexHull : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ConvexHull(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _mode("Mode", 0, 0, 3, parameters_,
              "0=CV_RETR_EXTERNAL,1=CV_RETR_LIST, 2=CV_RETR_CCOMP, "
              "3=CV_RETR_TREE"),
        _method("Method", 0, 0, 3, parameters_,
                "0=CV_CHAIN_APPROX_NONE, 1=CV_CHAIN_APPROX_SIMPLE, "
                "2=CV_CHAIN_APPROX_TC89_L1, "
                "3=CV_CHAIN_APPROX_TC89_KCOS") {
    setName("ConvexHull");
  }

  virtual ~ConvexHull() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      int mode, method;
      switch (_mode()) {
        case 0:
          mode = char(CV_RETR_EXTERNAL);
          break;
        case 1:
          mode = char(CV_RETR_LIST);
          break;
        case 2:
          mode = char(CV_RETR_CCOMP);
          break;
        case 3:
          mode = char(CV_RETR_TREE);
          break;
      }
      switch (_method()) {
        case 0:
          method = char(CV_CHAIN_APPROX_NONE);
          break;
        case 1:
          method = char(CV_CHAIN_APPROX_SIMPLE);
          break;
        case 2:
          method = char(CV_CHAIN_APPROX_TC89_L1);
          break;
        case 3:
          method = char(CV_CHAIN_APPROX_TC89_KCOS);
          break;
      }
      cv::vector<cv::vector<cv::Point> > contours;
      cv::vector<cv::Vec4i> hierarchy;

      // Find contours
      cv::findContours(image, contours, hierarchy, mode, method,
                       cv::Point(0, 0));

      // Find the convex hull object for each contour
      cv::vector<cv::vector<cv::Point> > hull(contours.size());
      for (size_t i = 0; i < contours.size(); i++) {
        cv::convexHull(cv::Mat(contours[i]), hull[i], false);
      }

      // Draw Hull contour
      image = cv::Mat::zeros(image.size(), CV_8UC1);
      for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(image, hull, i, cv::Scalar(255, 255, 255), CV_FILLED);
      }
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _mode, _method;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_CONVEX_HULL_H_
