/**
 * \file	object_finder.h
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

#ifndef LIB_VISION_FILTERS_OBJECT_FINDER_H_
#define LIB_VISION_FILTERS_OBJECT_FINDER_H_

#include <memory>
#include <lib_vision/filter.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/target.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/object_feature_factory.h>
#include <lib_vision/algorithm/performance_evaluator.h>

namespace lib_vision {

class ObjectFinder : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectFinder>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ObjectFinder(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        enable_("Enable", false, &parameters_),
        debug_contour_("Debug_contour", false, &parameters_),
        look_for_rectangle_("Look_for_Rectangle", false, &parameters_),
        disable_ratio_("disable_ratio_check", false, &parameters_),
        disable_angle_("disable_angle_check", false, &parameters_),
        use_convex_hull_("Use_convex_hull", false, &parameters_),
        vote_most_centered_("Vote_most_centered", false, &parameters_),
        vote_most_upright_("Vote_most_upright", false, &parameters_),
        vote_less_difference_from_targeted_ratio_(
            "Vote_less_diff_from_target_ratio", false, &parameters_),
        vote_length_("Vote_length", false, &parameters_),
        vote_higher_("Vote_higher", false, &parameters_),
        id_("ID", "buoy", &parameters_),
        spec_1_("spec1", "red", &parameters_),
        spec_2_("spec2", "blue", &parameters_),
        min_area_("Min_area", 200, 0, 10000, &parameters_),
        targeted_ratio_("Ratio_target", 0.5f, 0.0f, 1.0f, &parameters_),
        difference_from_target_ratio_("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, &parameters_),
        targeted_angle_("angle_target", 0.0f, 0.0f, 90.0f, &parameters_),
        difference_from_target_angle_("Diff_from_angle_target", 30.0f, 0.0f,
                                      90.0f, &parameters_),
        min_percent_filled_("Min_percent_filled", 50, 0, 100, &parameters_),
        contour_retreval_("Contour_retreval", 0, 0, 4, &parameters_,
                          "0=All, 1=Out, 2=Inner, 3=InnerMost, 4=OutNoChild"),
        feature_factory_(5) {
    SetName("ObjectFinder");
    // Little goodies for cvs
    // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
  }

  virtual ~ObjectFinder() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (enable_()) {
      if (debug_contour_()) {
        image.copyTo(output_image_);
        if (output_image_.channels() == 1) {
          cv::cvtColor(output_image_, output_image_, CV_GRAY2BGR);
        }
      }

      if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
      cv::Mat originalImage = global_params_.getOriginalImage();

      contourList_t contours;
      switch (contour_retreval_()) {
        case 0:
          RetrieveAllContours(image, contours);
          break;
        case 1:
          RetrieveOuterContours(image, contours);
          break;
        case 2:
          RetrieveAllInnerContours(image, contours);
          break;
        case 3:
          RetrieveInnerContours(image, contours);
          break;
        case 4:
          RetrieveOutNoChildContours(image, contours);
          break;
      }

      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
        if (use_convex_hull_()) {
          cv::convexHull(contours[i], contours[i]);
        }

        ObjectFullData::Ptr object =
            std::make_shared<ObjectFullData>(originalImage, image, contours[i]);
        if (object.get() == nullptr) {
          continue;
        }
        //
        // AREA
        //
        if (object->GetArea() < min_area_()) {
          continue;
        }
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(255, 0, 0), 2);
        }

        //
        // RATIO
        //
        if (!disable_ratio_() && (fabs(object->GetRatio() - targeted_ratio_()) >
                                  fabs(difference_from_target_ratio_()))) {
          continue;
        }
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(0, 0, 255), 2);
        }

        //
        // PERCENT FILLED
        //

        float percent_filled =
            CalculatePourcentFilled(image, object->GetUprightRect());
        if ((percent_filled) < min_percent_filled_()) {
          continue;
        }
        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(255, 255, 0), 2);
        }

        //
        // ANGLE
        //
        if (!disable_angle_() &&
            (fabs(object->GetRotatedRect().angle - targeted_angle_()) >
             fabs(difference_from_target_angle_()))) {
          continue;
        }

        //
        // RECTANGLE
        //
        if (look_for_rectangle_() && !IsRectangle(contours[i], 10)) {
          // if (look_for_rectangle_() && !IsSquare(contours[i], min_area_(),
          // 80.0f, 0.0f, 100.0f)) {
          continue;
        }

        if (debug_contour_()) {
          cv::drawContours(output_image_, contours, i, CV_RGB(0, 255, 0), 2);
        }

        objVec.push_back(object);
      }

      if (objVec.size() > 1) {
        if (vote_most_centered_()) {
          std::sort(
              objVec.begin(), objVec.end(),
              [this](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                return GetDistanceFromCenter(a) < GetDistanceFromCenter(b);
              });
          objVec[0]->IncrementVote();
          cv::circle(output_image_, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, 12),
                     6, CV_RGB(255, 0, 0), -1);
        }

        if (vote_length_()) {
          std::sort(objVec.begin(), objVec.end(),
                    [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                      return fabs(a->GetLength()) > fabs(b->GetLength());
                    });
          objVec[0]->IncrementVote();
          cv::circle(output_image_, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, 6),
                     6, CV_RGB(0, 0, 255), -1);
        }

        if (vote_most_upright_()) {
          std::sort(objVec.begin(), objVec.end(),
                    [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                      return fabs(a->GetRotatedRect().angle) <
                             fabs(b->GetRotatedRect().angle);
                    });
          objVec[0]->IncrementVote();
          cv::circle(output_image_, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, 0),
                     6, CV_RGB(255, 0, 255), -1);
        }

        if (vote_less_difference_from_targeted_ratio_()) {
          std::sort(
              objVec.begin(), objVec.end(),
              [this](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                return fabs(a->GetRatio() - targeted_ratio_()) <
                       fabs(b->GetRatio() - targeted_ratio_());
              });
          objVec[0]->IncrementVote();
          cv::circle(output_image_, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, -6),
                     6, CV_RGB(0, 255, 255), -1);
        }

        if (vote_higher_()) {
          std::sort(
              objVec.begin(), objVec.end(),
              [this](ObjectFullData::Ptr a, ObjectFullData::Ptr b)
                  -> bool { return a->GetCenter().y < b->GetCenter().y; });
          objVec[0]->IncrementVote();

          cv::circle(output_image_, cv::Point(objVec[0]->GetCenter().x,
                                              objVec[0]->GetCenter().y) -
                                        cv::Point(12, -12),
                     6, CV_RGB(255, 255, 0), -1);
        }

        std::sort(objVec.begin(), objVec.end(),
                  [](ObjectFullData::Ptr a, ObjectFullData::Ptr b)
                      -> bool { return a->GetArea() > b->GetArea(); });

        objVec[0]->IncrementVote();
        cv::circle(output_image_, cv::Point(objVec[0]->GetCenter().x,
                                            objVec[0]->GetCenter().y) -
                                      cv::Point(12, -18),
                   6, CV_RGB(255, 150, 150), -1);
      }

      std::sort(objVec.begin(), objVec.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b)
                    -> bool { return a->GetVoteCount() > b->GetVoteCount(); });

      if (objVec.size() > 0) {
        Target target;
        ObjectFullData::Ptr object = objVec[0];
        cv::Point center = object->GetCenter();
        target.SetTarget(id_(), center.x, center.y, object->GetLength(),
                         object->GetLength(), object->GetRotatedRect().angle,
        image.rows, image.cols);
        target.SetSpecField_1(spec_1_());
        target.SetSpecField_2(spec_2_());
        NotifyTarget(target);
        if (debug_contour_()) {
          cv::circle(output_image_, objVec[0]->GetCenter(), 3,
                     CV_RGB(0, 255, 0), 3);
        }
      }
      if (debug_contour_()) {
        output_image_.copyTo(image);
      }
    }
  }

  float GetDistanceFromCenter(ObjectFullData::Ptr object) {
    cv::Point center(object->GetBinaryImage().cols / 2,
                     object->GetBinaryImage().rows / 2);
    float x_diff = object->GetCenter().x - center.x;
    float y_diff = object->GetCenter().y - center.y;
    return x_diff * x_diff + y_diff * y_diff;
  };

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat output_image_;

  Parameter<bool> enable_, debug_contour_, look_for_rectangle_, disable_ratio_,
      disable_angle_, use_convex_hull_;

  Parameter<bool> vote_most_centered_, vote_most_upright_,
      vote_less_difference_from_targeted_ratio_, vote_length_, vote_higher_;

  Parameter<std::string> id_, spec_1_, spec_2_;

  RangedParameter<double> min_area_, targeted_ratio_,
      difference_from_target_ratio_, targeted_angle_,
      difference_from_target_angle_, min_percent_filled_;

  RangedParameter<int> contour_retreval_;

  ObjectFeatureFactory feature_factory_;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_OBJECT_FINDER_H_
