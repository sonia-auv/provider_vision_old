/**
 * \file	delorean_detector.h
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

#ifndef LIB_VISION_FILTERS_DELOREAN_DETECTOR_H_
#define LIB_VISION_FILTERS_DELOREAN_DETECTOR_H_

#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/filter.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/object_feature_factory.h>

namespace lib_vision {

class DeloreanDetector : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DeloreanDetector>;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit DeloreanDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _debug_contour("Debug_contour", false, parameters_),
        _output_train("Output_train", false, parameters_),
        _min_area("Min_area", 200, 0, 10000, parameters_),
        _targeted_ratio_big("Ratio_target_big", 0.5f, 0.0f, 1.0f, parameters_),
        _targeted_ratio_small("Ratio_target_small", 0.5f, 0.0f, 1.0f,
                              parameters_),
        _difference_from_target_ratio("Diff_from_ratio_target", 0.10f, 0.0f,
                                      1.0f, parameters_),
        _feat_factory(3) {
    setName("DeloreanDetector");
  }

  virtual ~DeloreanDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (image.channels() != 1) {
        cv::cvtColor(image, image, CV_BGR2GRAY);
      }

      if (_debug_contour()) {
        image.copyTo(_output_image);
        if (_output_image.channels() == 1) {
          cv::cvtColor(_output_image, _output_image, CV_GRAY2BGR);
        }
      }

      cv::Mat originalImage = global_params_.getOriginalImage();
      contourList_t contours;
      hierachy_t hierachy;
      retrieveOuterContours(image, contours);
      ObjectFullData::FullObjectPtrVec objVec_big;
      ObjectFullData::FullObjectPtrVec objVec_small;
      for (size_t i = 0, size = contours.size(); i < size; i++) {
        ObjectFullData::Ptr object =
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
          cv::drawContours(_output_image, contours, int(i), CV_RGB(255, 0, 0),
                           2);
        }

        //
        // RATIO
        //
        //        std::stringstream ss;
        //        ss << object->GetRatio();
        //        cv::putText(_output_image, ss.str(), object->GetCenter(),
        //                    cv::FONT_HERSHEY_SIMPLEX, 1 /*fontscale*/,
        //                    cv::Scalar(255, 0, 0), 3 /*thickness*/, 6
        //                    /*lineType*/,
        //                    false);
        _feat_factory.RatioFeature(object);
        double ratio_difference_big =
            fabs(object->GetRatio() - _targeted_ratio_big());
        double ratio_difference_small =
            fabs(object->GetRatio() - _targeted_ratio_small());
        //				std::cout << object->GetRatio() << " "
        //<<
        //_targeted_ratio() << " " << ratio << std::endl;

        // Could change for check for wich is nearest the target ratio, i.e. is
        // the ratio nearrer big object or
        // small object...
        if (ratio_difference_big > _difference_from_target_ratio() &&
            ratio_difference_small < _difference_from_target_ratio()) {
          objVec_small.push_back(object);
        } else if (ratio_difference_big < _difference_from_target_ratio() &&
                   ratio_difference_small > _difference_from_target_ratio()) {
          objVec_big.push_back(object);
        } else if (ratio_difference_big < _difference_from_target_ratio() &&
                   ratio_difference_small < _difference_from_target_ratio()) {
          objVec_big.push_back(object);

        } else {
          continue;
        }
        //				if( !IsRectangle( contours[i] ) )
        //				{
        //					continue;
        //				}
        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, int(i), CV_RGB(0, 0, 255),
                           2);
        }
      }

      std::sort(objVec_big.begin(), objVec_big.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b)
                    -> bool { return a->GetArea() > b->GetArea(); });
      std::sort(objVec_small.begin(), objVec_small.end(),
                [](ObjectFullData::Ptr a, ObjectFullData::Ptr b)
                    -> bool { return a->GetArea() > b->GetArea(); });

      if (objVec_big.size() > 0) {
        Target target;
        ObjectFullData::Ptr object_big = objVec_big[0];
        cv::Point center_big = object_big->GetCenter();
        double angle = 181;
        if (objVec_small.size() > 0) {
          ObjectFullData::Ptr object_small = objVec_small[0];
          cv::Point center_small = object_small->GetCenter();
          if (_debug_contour()) {
            cv::circle(_output_image, center_small, 2, CV_RGB(255, 0, 255), 2);
          }
          angle = 360 * (atan2(center_big.y - center_big.y,
                               center_big.x + 5 - center_big.x) -
                         atan2(center_small.y - center_big.y,
                               center_small.x - center_big.x)) /
                  (2 * 3.1416);
        }
        setCameraOffset(&center_big, image.rows, image.cols);
        target.SetTarget(center_big.x, center_big.y, object_big->GetLength(),
                         object_big->GetLength(), float(angle));
        std::stringstream ss;
        if (_output_train()) {
          ss << "train:" << target.OutputString();
        } else {
          ss << "delorean:" << target.OutputString();
        }
        notify_str(ss.str().c_str());
        if (_debug_contour()) {
          cv::circle(_output_image, objVec_big[0]->GetCenter(), 3,
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
  BooleanParameter _enable, _debug_contour, _output_train;
  DoubleParameter _min_area, _targeted_ratio_big, _targeted_ratio_small,
      _difference_from_target_ratio;
  ObjectFeatureFactory _feat_factory;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_DELOREAN_DETECTOR_H_
