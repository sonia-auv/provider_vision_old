/**
 * \file	PipeDetector.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_PIPE_DETECTOR_H_
#define VISION_FILTER_PIPE_DETECTOR_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <lib_vision/algorithm/features.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/performance_evaluator.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class PipeDetector : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit PipeDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _debug_contour("Debug_contour", false, parameters_),
        _look_for_rectangle("Look_for_Rectangle", false, parameters_),
        _min_area("Min_area", 200, 0, 10000, parameters_) {
    setName("PipeDetector");
  }

  virtual ~PipeDetector() {}

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

      PerformanceEvaluator timer;
      timer.UpdateStartTime();

      contourList_t contours;
      retrieveAllContours(image, contours);
      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
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
        // RECTANGLE
        //
        if (_look_for_rectangle() && !IsRectangle(contours[i], 10)) {
          continue;
        }
        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, i, CV_RGB(0, 255, 0), 2);
        }

        objVec.push_back(object);
      }

      std::sort(objVec.begin(), objVec.end(),
                [](std::shared_ptr<ObjectFullData> a,
                   std::shared_ptr<ObjectFullData> b)
                    -> bool { return a->GetArea() > b->GetArea(); });

      // Since we search only one buoy, get the biggest from sort function
      if (objVec.size() > 0) {
        Target target;
        std::shared_ptr<ObjectFullData> object = objVec[0];
        cv::Point center = object->GetCenter();
        setCameraOffset(&center, image.rows, image.cols);
        target.SetTarget(center.x, center.y, object->GetLength(),
                         object->GetLength(), object->GetRotatedRect().angle);
        std::stringstream ss;
        ss << "pipe:" << target.OutputString();
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
  BooleanParameter _enable, _debug_contour, _look_for_rectangle;
  DoubleParameter _min_area;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_PIPE_DETECTOR_H_
