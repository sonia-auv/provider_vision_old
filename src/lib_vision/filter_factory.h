/**
 * \file	filter_factory.h
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


#ifndef VISION_FILTER_FILTER_FACTORY_H_
#define VISION_FILTER_FILTER_FACTORY_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <string>

// All filter files are included here
#include <lib_vision/filters/blurr.h>
#include <lib_vision/filters/dilate.h>
#include <lib_vision/filters/erode.h>
#include <lib_vision/filters/mission_test_fake_string.h>
#include <lib_vision/filters/test_filter.h>
#include <lib_vision/filters/buoy_single.h>
#include <lib_vision/filters/morphology.h>
#include <lib_vision/filters/original_image.h>
#include <lib_vision/filters/scharr.h>
#include <lib_vision/filters/schar_adding.h>
#include <lib_vision/filters/stats_threshold.h>
#include <lib_vision/filters/subtract_all_planes.h>
#include <lib_vision/filters/threshold.h>
#include <lib_vision/filters/rotate.h>
#include <lib_vision/filters/fence_detector.h>
#include <lib_vision/filters/image_accumulator.h>
#include <lib_vision/filters/object_feature_calculator.h>
#include <lib_vision/filters/train_detector.h>
#include <lib_vision/filters/object_finder.h>
#include <lib_vision/filters/pipe_detector.h>
#include <lib_vision/filters/track_detector.h>
#include <lib_vision/filters/sobel.h>
#include <lib_vision/filters/delorean_detector.h>
#include <lib_vision/filters/submarine_frame_masker.h>
#include <lib_vision/filters/in_range.h>
#include <lib_vision/filters/convex_hull.h>
#include <lib_vision/filters/torpedoes_detector.h>
#include <lib_vision/filters/laplacian.h>
#include <lib_vision/filters/canny.h>
#include <lib_vision/filters/hough_line.h>
#include <lib_vision/filters/adaptive_threshold.h>
#include <lib_vision/filters/handle_detector.h>
#include <lib_vision/filters/white_noise_takedown.h>
#include <lib_vision/filters/bilateral_filter.h>

namespace lib_vision {

//==============================================================================
// C L A S S E S

// Class that provides an interface
// for the lib_vision project.
// It enables instantiation via a string
// and holds the list of all the filters.
class FilterFactory {
 public:
  // KEEPING A REFERENCE TO GlobalParamHandler. VERY IMPORTANT
  static Filter *createInstance(const std::string &name,
                                const GlobalParamHandler &globalParams);

  static std::string GetFilterList();
};

}  // namespace lib_vision

#endif  // VISION_FILTER_FILTER_FACTORY_H_
