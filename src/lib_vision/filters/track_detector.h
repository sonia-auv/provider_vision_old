/**
 * \file	track_detector.h
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

#ifndef LIB_VISION_FILTERS_TRACK_DETECTOR_H_
#define LIB_VISION_FILTERS_TRACK_DETECTOR_H_

#include <vector>
#include <memory>
#include <lib_vision/filter.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/object_feature_factory.h>
#include <lib_vision/algorithm/type_and_const.h>

namespace lib_vision {

class TrackDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TrackDetector>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit TrackDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _debug_contour("Debug_contour", false, &parameters_),
        _min_area("Min_area", 200, 0, 10000, &parameters_),
        _targeted_ratio("Ratio_target", 0.5f, 0.0f, 1.0f, &parameters_),
        _difference_from_target_ratio("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, &parameters_),
        _feat_factory(3) {
    SetName("TrackDetector");
  }

  virtual ~TrackDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (_enable()) {
      if (_debug_contour()) {
        image.copyTo(_output_image);
        if (_output_image.channels() == 1) {
          cv::cvtColor(_output_image, _output_image, CV_GRAY2BGR);
        }
      }
      if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);

      cv::Mat originalImage = global_params_.getOriginalImage();
      contourList_t contours, inner_most_contour;
      hierachy_t hierachy;
      // Should optimize to find only one time and parse afterward...
      retrieveOuterContours(image, contours);
      retrieveInnerContours(image, inner_most_contour);
      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
        contour_t hull;
        cv::convexHull(contours[i], hull, false);
        ObjectFullData::Ptr object =
            std::make_shared<ObjectFullData>(originalImage, image, hull);
        if (object.get() == nullptr) {
          continue;
        }
        //
        // AREA
        //
        if (object->GetArea() < _min_area()) {
          continue;
        }
        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, i, CV_RGB(255, 0, 0), 2);
        }

        objVec.push_back(object);
      }

      std::sort(objVec.begin(), objVec.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b)
                    -> bool { return a->GetArea() > b->GetArea(); });

      // Get all the square contours.
      contourList_t squareContour;
      for (auto innerContour : inner_most_contour) {
        if (cv::contourArea(innerContour) < _min_area()) {
          continue;
        }

        if (IsRectangle(innerContour, 10)) {
          squareContour.push_back(innerContour);
        }
      }
      if (_debug_contour()) {
        cv::drawContours(_output_image, squareContour, -1, CV_RGB(255, 0, 255),
                         3);
      }

      // Votes for the contour with the most
      std::vector<std::pair<ObjectFullData::Ptr, int> > contour_vote;
      for (auto &square : squareContour) {
        for (auto &already_voted_for : contour_vote) {
          if (cv::pointPolygonTest(
                  already_voted_for.first->GetContourCopy().Get(),
                  cv::Point2f(square[0].x, square[0].y), false) > 0.0f) {
            already_voted_for.second++;
            continue;
          }
        }

        for (auto &to_added_to_the_voted_pool : objVec) {
          if (cv::pointPolygonTest(
                  to_added_to_the_voted_pool->GetContourCopy().Get(),
                  cv::Point2f(square[0].x, square[0].y), false) > 0.0f) {
            contour_vote.push_back(std::pair<ObjectFullData::Ptr, int>(
                to_added_to_the_voted_pool, 1));
            cv::polylines(_output_image,
                          to_added_to_the_voted_pool->GetContourCopy().Get(),
                          true, CV_RGB(255, 0, 255), 3);
            continue;
          }
        }
      }
      std::sort(contour_vote.begin(), contour_vote.end(),
                [](const std::pair<ObjectFullData::Ptr, int> &a,
                   const std::pair<ObjectFullData::Ptr, int> &b)
                    -> bool { return a.second > b.second; });

      // Since we search only one buoy, get the biggest from sort function
      if (contour_vote.size() > 0) {
        Target target;
        ObjectFullData::Ptr object = contour_vote[0].first;
        cv::Point center = object->GetCenter();
        setCameraOffset(&center, image.rows, image.cols);
        target.SetTarget(center.x, center.y, object->GetLength(),
                         object->GetLength(), object->GetRotatedRect().angle);
        std::stringstream ss;
        ss << "track:" << target.OutputString();
        NotifyString(ss.str().c_str());
        if (_debug_contour()) {
          cv::circle(_output_image, objVec[0]->GetCenter(), 3,
                     CV_RGB(0, 255, 0), 3);
        }
      }

      if (_debug_contour()) {
        _output_image.copyTo(image);
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat _output_image;
  // Params
  Parameter<bool> _enable, _debug_contour;
  RangedParameter<double> _min_area, _targeted_ratio, _difference_from_target_ratio;

  ObjectFeatureFactory _feat_factory;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_TRACK_DETECTOR_H_
