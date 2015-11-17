/**
 * \file	stats_threshold.h
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


#ifndef VISION_FILTER_STATS_THRESHOLD_H_
#define VISION_FILTER_STATS_THRESHOLD_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

class StatsThreshold : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit StatsThreshold(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _min_thresh("Min_thresh", 0, 0, 255, parameters_),
        _mean_multiplier("Mean_multiplier", 1, -10, 10, parameters_),
        _std_dev_multiplier("Standard_deviation_multiplier", 1, -10, 10,
                            parameters_)
        {
    setName("StatsThreshold");
  }

  virtual ~StatsThreshold() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() > 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      if (image.depth() != CV_8U) {
        image.convertTo(image, CV_8U);
      }
      cv::Scalar mean, stdDev;

      cv::meanStdDev(image, mean, stdDev);
      int thresh_val =
          mean[0] * _mean_multiplier() + stdDev[0] * _std_dev_multiplier();
      thresh_val = thresh_val < _min_thresh() ? _min_thresh() : thresh_val;
      cv::threshold(image, image, thresh_val, 255, CV_THRESH_BINARY);
    }
  }

 private:
  // Params
  BooleanParameter _enable;
  IntegerParameter _min_thresh;
  DoubleParameter _mean_multiplier, _std_dev_multiplier;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_STATS_THRESHOLD_H_
