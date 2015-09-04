/**
 * \file	ObjectFinder.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_OBJECT_FINDER_H_
#define VISION_FILTER_OBJECT_FINDER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <lib_vision/algorithm/features.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/feature_factory.h>
#include <lib_vision/algorithm/performance_evaluator.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class ObjectFinder : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ObjectFinder(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _debug_contour("Debug_contour", false, parameters_),
        _look_for_rectangle("Look_for_Rectangle", false, parameters_),
        _disable_ratio("disable_ratio_check", false, parameters_),
        _disable_angle("disable_angle_check", false, parameters_),
        _id("ID", "buoy", parameters_),
        _spec_1("spec1", "red", parameters_),
        _spec_2("spec2", "blue", parameters_),
        _min_area("Min_area", 200, 0, 10000, parameters_),
        _targeted_ratio("Ratio_target", 0.5f, 0.0f, 1.0f, parameters_),
        _difference_from_target_ratio("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, parameters_),
        _targeted_angle("angle_target", 0.0f, 0.0f, 90.0f, parameters_),
        _difference_from_target_angle("Diff_from_angle_target", 30.0f, 0.0f,
                                      90.0f, parameters_),
        _contour_retreval("Contour_retreval", 0, 0, 4, parameters_,
                          "0=All, 1=Out, 2=Inner, 3=InnerMost, 4=OutNoChild"),
        _min_percent_filled("Min_percent_filled", 50, 0, 100, parameters_),
        _use_convex_hull("Use_convex_hull", false, parameters_),
        _vote_most_centered("Vote_most_centered", false, parameters_),
        _vote_most_upright("Vote_most_upright", false, parameters_),
        _vote_less_difference_from_targeted_ratio(
            "Vote_less_diff_from_target_ratio", false, parameters_),
        _vote_length("Vote_length", false, parameters_),
        _vote_higher("Vote_higher", false, parameters_),
        _feature_factory(5) {
    setName("ObjectFinder");
    _feature_factory.SetAllFeatureToCompute();
    // Little goodies for cvs
    // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
  }

  virtual ~ObjectFinder() {}

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

      contourList_t contours;
      switch (_contour_retreval()) {
        case 0:
          retrieveAllContours(image, contours);
          break;
        case 1:
          retrieveOuterContours(image, contours);
          break;
        case 2:
          retrieveAllInnerContours(image, contours);
          break;
        case 3:
          retrieveInnerContours(image, contours);
          break;
        case 4:
          retrieveOutNoChildContours(image, contours);
          break;
      }

      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
        if (_use_convex_hull()) {
          cv::convexHull(contours[i], contours[i]);
        }

        std::shared_ptr<ObjectFullData> object =
            std::make_shared<ObjectFullData>(originalImage, image, contours[i]);
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

        //
        // RATIO
        //
        if (!_disable_ratio() && (fabs(object->GetRatio() - _targeted_ratio()) >
                                  fabs(_difference_from_target_ratio()))) {
          continue;
        }
        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, i, CV_RGB(0, 0, 255), 2);
        }

        //
        // PERCENT FILLED
        //

        float percent_filled =
            calculatePourcentFilled(image, object->GetUprightRect());
        if ((percent_filled) < _min_percent_filled()) {
          continue;
        }
        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, i, CV_RGB(255, 255, 0), 2);
        }

        //
        // ANGLE
        //
        if (!_disable_angle() &&
            (fabs(object->GetRotatedRect().angle - _targeted_angle()) >
             fabs(_difference_from_target_angle()))) {
          continue;
        }

        //
        // RECTANGLE
        //
        if (_look_for_rectangle() && !IsRectangle(contours[i], 10)) {
          // if (_look_for_rectangle() && !IsSquare(contours[i], _min_area(),
          // 80.0f, 0.0f, 100.0f)) {
          continue;
        }

        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, i, CV_RGB(0, 255, 0), 2);
        }

        objVec.push_back(object);
      }

      if (objVec.size() > 1) {
        if (_vote_most_centered()) {
          std::sort(objVec.begin(), objVec.end(),
                    [](std::shared_ptr<ObjectFullData> a,
                       std::shared_ptr<ObjectFullData> b) -> bool {
                      return a->GetDistanceFromCenter() <
                             b->GetDistanceFromCenter();
                    });
          objVec[0]->IncrementVote();
          cv::circle(_output_image, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, 12),
                     6, CV_RGB(255, 0, 0), -1);
        }

        if (_vote_length()) {
          std::sort(objVec.begin(), objVec.end(),
                    [](std::shared_ptr<ObjectFullData> a,
                       std::shared_ptr<ObjectFullData> b) -> bool {
                      return fabs(a->GetLength()) > fabs(b->GetLength());
                    });
          objVec[0]->IncrementVote();
          cv::circle(_output_image, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, 6),
                     6, CV_RGB(0, 0, 255), -1);
        }

        if (_vote_most_upright()) {
          std::sort(objVec.begin(), objVec.end(),
                    [](std::shared_ptr<ObjectFullData> a,
                       std::shared_ptr<ObjectFullData> b) -> bool {
                      return fabs(a->GetRotatedRect().angle) <
                             fabs(b->GetRotatedRect().angle);
                    });
          objVec[0]->IncrementVote();
          cv::circle(_output_image, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, 0),
                     6, CV_RGB(255, 0, 255), -1);
        }

        if (_vote_less_difference_from_targeted_ratio()) {
          std::sort(objVec.begin(), objVec.end(),
                    [this](std::shared_ptr<ObjectFullData> a,
                           std::shared_ptr<ObjectFullData> b) -> bool {
                      return fabs(a->GetRatio() - this->_targeted_ratio()) <
                             fabs(b->GetRatio() - this->_targeted_ratio());
                    });
          objVec[0]->IncrementVote();
          cv::circle(_output_image, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, -6),
                     6, CV_RGB(0, 255, 255), -1);
        }

        if (_vote_higher()) {
          std::sort(objVec.begin(), objVec.end(),
                    [this](std::shared_ptr<ObjectFullData> a,
                           std::shared_ptr<ObjectFullData> b) -> bool {
                      return a->GetCenter().y < b->GetCenter().y;
                    });
          objVec[0]->IncrementVote();

          cv::circle(_output_image, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, -12),
                     6, CV_RGB(255, 255, 0), -1);
        }

        std::sort(objVec.begin(), objVec.end(),
                  [](std::shared_ptr<ObjectFullData> a,
                     std::shared_ptr<ObjectFullData> b)
                      -> bool { return a->GetArea() > b->GetArea(); });

        objVec[0]->IncrementVote();
        cv::circle(_output_image, cv::Point(objVec[0]->GetCenter().x,
                                            objVec[0]->GetCenter().y) -
                                      cv::Point(12, -18),
                   6, CV_RGB(255, 150, 150), -1);
      }

      std::sort(objVec.begin(), objVec.end(),
                [](std::shared_ptr<ObjectFullData> a,
                   std::shared_ptr<ObjectFullData> b)
                    -> bool { return a->GetVoteCount() > b->GetVoteCount(); });

      if (objVec.size() > 0) {
        Target target;
        std::shared_ptr<ObjectFullData> object = objVec[0];
        cv::Point center = object->GetCenter();
        setCameraOffset(&center, image.rows, image.cols);
        target.setTarget(center.x, center.y, object->GetLength(),
                         object->GetLength(), object->GetRotatedRect().angle);
        target.setSpecField_1(_spec_1());
        target.setSpecField_2(_spec_2());
        std::stringstream ss;
        ss << _id() << target.outputString();
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
  BooleanParameter _enable, _debug_contour, _look_for_rectangle, _disable_ratio,
      _disable_angle, _use_convex_hull;
  BooleanParameter _vote_most_centered, _vote_most_upright,
      _vote_less_difference_from_targeted_ratio, _vote_length, _vote_higher;
  StringParameter _id, _spec_1, _spec_2;
  DoubleParameter _min_area, _targeted_ratio, _difference_from_target_ratio,
      _targeted_angle, _difference_from_target_angle, _min_percent_filled;
  IntegerParameter _contour_retreval;

  FeatureFactory _feature_factory;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_OBJECT_FINDER_H_
