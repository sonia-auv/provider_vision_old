/**
 * \file	BuoySingle.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_BUOY_SINGLE_H_
#define VISION_FILTER_BUOY_SINGLE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <lib_vision/algorithm/features.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <math.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class BuoySingle : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit BuoySingle(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _debug_good_contour("Debug_contour", false, parameters_),
        _eliminate_same_x_targets("Eliminate_same_x", false, parameters_),
        _color("Buoy", "red", parameters_),
        _min_area("Min_area", 200, 0, 10000, parameters_),
        _max_ratio("Max_ratio", 50, 0, 100, parameters_),
        _min_filled_percent("Min_percent", 50, 0, 100, parameters_),
        _max_x_difference_for_elimination("Min_x_difference", 50.0f, 0.0f,
                                          1000.0f, parameters_),
        _detect_red("Detect_red", false, parameters_),
        _ratio_for_angle_check(
            "Ratio_for_angle_check", 50, 0, 100, parameters_,
            "When ratio smaller, will discard if vertical contour"),
        _admissible_horizontal_angle(
            "horizontal_max_angle", 50, 0, 90, parameters_,
            "When angle smaller, is consider vertical") {
    setName("BuoySingle");
  }

  virtual ~BuoySingle() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      cv::Mat in;
      // Copy the received image so we can see the objects.
      if (_debug_good_contour()) {
        // Case we receive a color or gray scale image.
        if (image.channels() == 1)
          cv::cvtColor(image, _outputImage, CV_GRAY2BGR);
        else
          image.copyTo(_outputImage);
      }

      if (image.channels() > 1) {
        cv::cvtColor(image, in, CV_BGR2GRAY);
      } else {
        image.copyTo(in);
      }
      // find contours in the image
      contourList_t contours;
      retrieveContours(in, contours);

      std::vector<Features> featuresVect;

      // Filter contours
      for (int j = 0; j < contours.size(); j++) {
        if (contours[j].size() <= 2) continue;

        Features temp(contours[j], in, Features::DESC_BASIC);

        // AREA
        if (_min_area() > temp.getArea()) {
          continue;
        }

        if (_debug_good_contour()) {
          temp.drawFeature(_outputImage, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0),
                           3);
        }

        temp.UpgradeToStandard();

        // RATIO
        if (_max_ratio() > temp.getRatio()) {
          continue;
        }

        if (_debug_good_contour()) {
          temp.drawFeature(_outputImage, CV_RGB(150, 150, 0),
                           CV_RGB(255, 255, 0), 3);
        }

        // Angle of contour. If more rectangular, check to make sure it is
        // horizontal
        // not vertical, horizontal is caused by washed out buoy in the sun,
        // while
        // vertical is probably a pipe

        if (temp.getRatio() < _ratio_for_angle_check() &&
            fabs(temp.getAngle()) < _admissible_horizontal_angle()) {
          continue;
        }

        if (_debug_good_contour()) {
          temp.drawFeature(_outputImage, CV_RGB(255, 255, 0),
                           CV_RGB(255, 255, 0), 3);
        }

        cv::Rect rect = cv::boundingRect(contours[j]);
        float pourcentFilled = calculatePourcentFilled(in, rect);
        if (_min_filled_percent() > pourcentFilled) {
          continue;
        }

        featuresVect.push_back(temp);
      }

      // Gets the contour that is rounder and bigger
      std::sort(featuresVect.begin(), featuresVect.end(), areaSort);

      if (_eliminate_same_x_targets()) {
        std::vector<unsigned int> index_to_eliminate;
        //          std::cout << "New frame " << featuresVect[0].getArea() << "
        //          " << featuresVect[1].getArea() << " " <<
        //          featuresVect[2].getArea() << std::endl;
        // We should not have much target, so double loop is ok...
        for (unsigned int i = 0, size = featuresVect.size(); i < size; i++) {
          for (unsigned int j = 0; j < size; j++) {
            if (i == j) {
              continue;
            }

            //              std::cout << "i " << featuresVect[i].center << "\t j
            //              " << featuresVect[j].center
            //                  << "\t IsSameX " << IsSameX(featuresVect[i],
            //                  featuresVect[j]) << "\t IsHigher "
            //                  << IsHigher( featuresVect[i], featuresVect[j] )
            //                  << std::endl;
            if (IsSameX(featuresVect[i], featuresVect[j])) {
              // If I is higher, eliminate it
              if (IsHigher(featuresVect[i], featuresVect[j])) {
                index_to_eliminate.push_back(i);
              }
            }
          }
        }

        if (index_to_eliminate.size() > 0) {
          for (int i = index_to_eliminate.size() - 1; i >= 0; i--) {
            featuresVect.erase(featuresVect.begin() + i);
          }
        }
      }

      // Choose red if need be
      if (_detect_red() && featuresVect.size() > 1) {
        Features a = featuresVect[0], b = featuresVect[1];
        int nb_test_point = 5;
        float area_diff = abs(a.getArea() - b.getArea()) / a.getArea();
        cv::Mat original_img, hsv;
        cv::cvtColor(global_params_.getOriginalImage(), hsv, CV_BGR2HSV);
        cv::copyMakeBorder(hsv, hsv, 21, 21, 21, 21, cv::BORDER_CONSTANT);
        cv::Mat out;
        cv::cvtColor(image, out, CV_GRAY2BGR);
        cv::Point center_a = featuresVect[0].getCenter(),
                  center_b = featuresVect[1].getCenter();

        cv::Mat roiA(hsv, cv::Rect(cv::Point(center_a.x - 5, (center_a.y)),
                                   cv::Size(20, 20)));
        cv::Mat roiB(hsv, cv::Rect(cv::Point(center_b.x - 5, (center_b.y)),
                                   cv::Size(20, 20)));

        cv::Scalar meanA = cv::mean(roiA);
        cv::Scalar meanB = cv::mean(roiB);

        // If a is more green, then it is yellow so we swap
        if (meanB[0] > meanA[0]) {
          std::swap(featuresVect[0], featuresVect[1]);
        }
      }

      // Since we search only one buoy, get the biggest from sort function
      if (featuresVect.size() != 0) {
        Target buoy;
        std::stringstream message;
        buoy.setTarget(featuresVect[0]);
        message << "buoy_" << _color() << ":" << buoy.outputString();
        notify_str(message.str());

        if (_debug_good_contour()) {
          featuresVect[0].drawFeature(_outputImage, CV_RGB(0, 255, 0),
                                      CV_RGB(0, 255, 0), 3);
        }
      }
      if (_debug_good_contour()) {
        _outputImage.copyTo(image);
      }
    }
  }

 private:
  inline float getRadiusFromRectangle(const Features &rectangle) {
    return (rectangle.size.width / 2 + rectangle.size.height / 2) / 2;
  }

  inline bool IsSameX(const Features &a, const Features &b) {
    double x_difference =
        static_cast<double>(a.center.x) - static_cast<double>(b.center.x);
    double abs_x_difference = fabs(x_difference);
    return abs_x_difference < _max_x_difference_for_elimination();
  }

  // check if ref is higher than compared
  inline bool IsHigher(const Features &ref, const Features &compared) {
    return ref.center.y < compared.center.y;
  }

  cv::Mat _outputImage;

  // Params
  BooleanParameter _enable, _debug_good_contour, _eliminate_same_x_targets,
      _detect_red;
  StringParameter _color;
  DoubleParameter _min_area, _max_ratio, _min_filled_percent,
      _max_x_difference_for_elimination, _ratio_for_angle_check,
      _admissible_horizontal_angle;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_BUOY_SINGLE_H_
