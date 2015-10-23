/**
 * \file	TrackDetector.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_TRACK_DETECTOR_H_
#define VISION_FILTER_TRACK_DETECTOR_H_

//==============================================================================
// I N C L U D E   F I L E S
#include <vector>
#include <memory>
#include <lib_vision/filter.h>
#include <lib_vision/algorithm/features.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/object_feature_factory.h>
#include <lib_vision/algorithm/type_and_const.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class TrackDetector : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit TrackDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _debug_contour("Debug_contour", false, parameters_),
        _min_area("Min_area", 200, 0, 10000, parameters_),
        _targeted_ratio("Ratio_target", 0.5f, 0.0f, 1.0f, parameters_),
        _difference_from_target_ratio("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, parameters_),
        _feat_factory(3) {
    setName("TrackDetector");
  }

  virtual ~TrackDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
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
        std::shared_ptr<ObjectFullData> object =
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
                [](std::shared_ptr<ObjectFullData> a,
                   std::shared_ptr<ObjectFullData> b)
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
      std::vector<std::pair<std::shared_ptr<ObjectFullData>, int> >
          contour_vote;
      for (auto &square : squareContour) {
        for (auto &already_voted_for : contour_vote) {
          if (cv::pointPolygonTest(already_voted_for.first->GetContourCopy(),
                                   cv::Point2f(square[0].x, square[0].y),
                                   false) > 0.0f) {
            already_voted_for.second++;
            continue;
          }
        }

        for (auto &to_added_to_the_voted_pool : objVec) {
          if (cv::pointPolygonTest(to_added_to_the_voted_pool->GetContourCopy(),
                                   cv::Point2f(square[0].x, square[0].y),
                                   false) > 0.0f) {
            contour_vote.push_back(
                std::pair<std::shared_ptr<ObjectFullData>, int>(
                    to_added_to_the_voted_pool, 1));
            cv::polylines(_output_image,
                          to_added_to_the_voted_pool->GetContourCopy(), true,
                          CV_RGB(255, 0, 255), 3);
            continue;
          }
        }
      }
      std::sort(contour_vote.begin(), contour_vote.end(),
                [](const std::pair<std::shared_ptr<ObjectFullData>, int> &a,
                   const std::pair<std::shared_ptr<ObjectFullData>, int> &b)
                    -> bool { return a.second > b.second; });

      // Since we search only one buoy, get the biggest from sort function
      if (contour_vote.size() > 0) {
        Target target;
        std::shared_ptr<ObjectFullData> object = contour_vote[0].first;
        cv::Point center = object->GetCenter();
        setCameraOffset(&center, image.rows, image.cols);
        target.SetTarget(center.x, center.y, object->GetLength(),
                         object->GetLength(), object->GetRotatedRect().angle);
        std::stringstream ss;
        ss << "track:" << target.OutputString();
        notify_str(ss.str().c_str());
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
  cv::Mat _output_image;
  // Params
  BooleanParameter _enable, _debug_contour;
  DoubleParameter _min_area, _targeted_ratio, _difference_from_target_ratio;

  FeatureFactory _feat_factory;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_TRACK_DETECTOR_H_
