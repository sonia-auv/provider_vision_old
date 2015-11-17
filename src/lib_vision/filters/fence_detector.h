/**
 * \file	fence_detector.h
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

#ifndef VISION_FILTER_FENCE_DETECTOR_H_
#define VISION_FILTER_FENCE_DETECTOR_H_

#include <lib_vision/filter.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/algorithm/object_feature_factory.h>

namespace lib_vision {

class FenceDetector : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit FenceDetector(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, parameters_),
        _debug_contour("Debug_contour", false, parameters_),
        _search_only_bottom("Search_only_bottom", false, parameters_,
                            "Enables searching only for bottom bar"),
        _min_length("Minimum_length", 50, 0, 2000, parameters_),
        _max_distance_from_bottom_bar_extremum("Max_dist_from_extremum", 50, 0,
                                               2000, parameters_),
        _min_area("Minimum_area", 300, 0, 10000, parameters_),
        _max_diff_from_90_tbca_horizontal(
            "Max_diff_horizontal", 20, 0, 90, parameters_,
            "Maximum difference from 90 to be consider as horizontal"),
        _max_diff_from_0_tbca_vertical(
            "Max_diff_vertical", 20, 0, 90, parameters_,
            "Maximum difference from 0 to be consider as vertical"),
        _min_percent_filled("Minimum_percent_filled", 70, 0, 1000, parameters_),
        _feat_factory(3) {
    setName("FenceDetector");
  }

  virtual ~FenceDetector() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (!_enable()) {
      return;
    }

    cv::Mat in;
    if (_debug_contour()) {
      // Case we receive a color or gray scale image.
      if (image.channels() == 1) {
        cv::cvtColor(image, _output_image, CV_GRAY2BGR);
      } else {
        image.copyTo(_output_image);
      }
    }

    if (image.channels() != 1) {
      cv::cvtColor(image, in, CV_BGR2GRAY);
    } else {
      image.copyTo(in);
    }

    contourList_t contours;
    retrieveOuterContours(in, contours);
    std::vector<ObjectFullData::Ptr> verticalBars, horizontalBar,
        merged_horizontal_bar;

    cv::Mat originalImage = global_params_.getOriginalImage();

    // Parse contours into 2 categories, vertical or horizontal.
    for (int i = 0, size = contours.size(); i < size; i++) {
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
        cv::drawContours(_output_image, contours, i, CV_RGB(255, 0, 0), 2);
      }

      //
      // LENGTH
      //
      if (object->GetRotatedRect().size.height < _min_length()) continue;
      if (_debug_contour()) {
        cv::drawContours(_output_image, contours, i, CV_RGB(255, 255, 0), 3);
      }
      _feat_factory.PercentFilledFeature(object);

      if (int(object->GetPercentFilled() * 100.0f) < _min_percent_filled())
        continue;
      if (_debug_contour()) {
        cv::drawContours(_output_image, contours, i, CV_RGB(255, 0, 255), 4);
      }

      float angle = fabs(object->GetRotatedRect().angle);

      if (angle < _max_diff_from_0_tbca_vertical()) {
        verticalBars.push_back(object);
      } else if ((90 - angle) < _max_diff_from_90_tbca_horizontal()) {
        horizontalBar.push_back(object);
      }
    }

    // Sort the bars with different criteria
    // Here we look for horizontal bar because it's
    std::sort(horizontalBar.begin(), horizontalBar.end(),
              [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                return a->GetRotatedRect().size.height >
                       b->GetRotatedRect().size.height;
              });

    if (horizontalBar.size() >= 2) {
      std::vector<std::pair<int, int>> pairs;
      for (int ref_idx = 0, size_ref = horizontalBar.size(); ref_idx < size_ref;
           ref_idx++) {
        if (IsSplitBar(horizontalBar[0], horizontalBar[1])) {
          contour_t tmp, a, b;
          a = horizontalBar[0]->GetContourCopy().Get();
          b = horizontalBar[1]->GetContourCopy().Get();

          tmp.reserve(a.size() + b.size());
          tmp.insert(tmp.end(), a.begin(), a.end());
          tmp.insert(tmp.end(), b.begin(), b.end());
          horizontalBar[0] =
              std::make_shared<ObjectFullData>(originalImage, image, tmp);
        }
      }
    }

    //
    // the easiest to find... if you did not find this one...
    // you probably didn't find any other... by experience.
    // Also, you need to return a size to AUV6 so...
    if (horizontalBar.size() != 0) {
      // Gets bottom bar info.
      ObjectFullData::Ptr final_horizontal_bar = horizontalBar[0];
      RotRect rect_from_hori_bar = final_horizontal_bar->GetRotatedRect();
      if (_debug_contour()) {
        cv::circle(_output_image, rect_from_hori_bar.center, 3,
                   CV_RGB(0, 0, 255), 3);
      }

      Target fence;
      cv::Point center = (rect_from_hori_bar.center);
      setCameraOffset(&center, image.rows, image.cols);
      fence.SetTarget(center.x, center.y, rect_from_hori_bar.size.width,
                      rect_from_hori_bar.size.height, rect_from_hori_bar.angle,
                      "", "");

      int y_coord_from_bottom = CalculateYFromBottomBar(
          rect_from_hori_bar.size.height, rect_from_hori_bar.center.y);

      if (_search_only_bottom()) {
        center = cv::Point(rect_from_hori_bar.center.x, y_coord_from_bottom);
        if (_debug_contour()) {
          cv::circle(_output_image, center, 5, CV_RGB(0, 255, 255), 20);
        }
        setCameraOffset(&center, image.rows, image.cols);

        fence.SetCenter(center);

      } else  // Gets the two best vertical bar to compute our y center.
      {
        std::sort(verticalBars.begin(), verticalBars.end(),
                  [](ObjectFullData::Ptr a, ObjectFullData::Ptr b) -> bool {
                    return a->GetRotatedRect().size.height >
                           b->GetRotatedRect().size.height;
                  });

        std::vector<cv::Point2f> final_vert_bar;
        int leftX, rightX;
        GetBottomBarXExtremum(final_horizontal_bar, leftX, rightX);

        // Extract the two major bar that are near the extremum of the bottom
        // post.
        int i = 0, size = verticalBars.size(), bar_founded = 0;
        for (; i < size && bar_founded != 2; i++) {
          if (IsNearExtremum(verticalBars[i]->GetRotatedRect().center.x, leftX,
                             rightX)) {
            final_vert_bar.push_back(verticalBars[i]->GetRotatedRect().center);
            if (_debug_contour()) {
              cv::circle(_output_image,
                         verticalBars[i]->GetRotatedRect().center, 5,
                         CV_RGB(0, 255, 255), 20);
            }
            bar_founded++;
          }
        }

        if (bar_founded == 2) {
          int x = 0, y = 0;
          // X from bottom bar plus
          x = (rect_from_hori_bar.center.x +
               (final_vert_bar[0].x + final_vert_bar[1].x) / 2) /
              2;

          y = (y_coord_from_bottom + final_vert_bar[0].y +
               final_vert_bar[1].y) /
              3;
          center = cv::Point(x, y);
          if (_debug_contour()) {
            cv::circle(_output_image, center, 5, CV_RGB(0, 255, 255), 20);
          }
          setCameraOffset(&center, image.rows, image.cols);
          fence.SetCenter(center);
        } else if (bar_founded == 1) {
          int y = (y_coord_from_bottom + final_vert_bar[0].y) / 2;
          center = cv::Point(rect_from_hori_bar.center.x, y);
          if (_debug_contour()) {
            cv::circle(_output_image, center, 5, CV_RGB(0, 255, 255), 20);
          }
          setCameraOffset(&center, image.rows, image.cols);
          fence.SetCenter(center);
        } else {
          center = cv::Point(rect_from_hori_bar.center.x, y_coord_from_bottom);
          if (_debug_contour()) {
            cv::circle(_output_image, center, 5, CV_RGB(0, 255, 255), 20);
          }
          setCameraOffset(&center, image.rows, image.cols);
          fence.SetCenter(center);
        }
      }

      std::stringstream message;
      message << "fence_big:" << fence.OutputString();
      notify_str(message.str());
    }
    if (_debug_contour()) {
      _output_image.copyTo(image);
    }
  }

 private:
  inline int CalculateYFromBottomBar(float bar_size, int bar_y) {
    int offset = static_cast<int>((bar_size) / 5.0f);
    return bar_y - offset;
  }

  inline void GetBottomBarXExtremum(ObjectFullData::Ptr bottom_bar, int &leftX,
                                    int &rightX) {
    leftX = 20000;
    rightX = -1;
    contour_t contour = bottom_bar->GetContourCopy().Get();
    for (auto pt : contour) {
      if (leftX > pt.x) {
        leftX = pt.x;
      }
      if (rightX < pt.x) {
        rightX = pt.x;
      }
    }
  }

  inline bool IsNearExtremum(int x_coord, int leftX, int rightX) {
    return IsBetweenLimit(x_coord, leftX) || IsBetweenLimit(x_coord, rightX);
  }

  inline bool IsBetweenLimit(int pt_x, int ref_x) {
    int left_max = (ref_x - _max_distance_from_bottom_bar_extremum());
    int right_max = (ref_x + _max_distance_from_bottom_bar_extremum());
    bool left_ok = left_max < pt_x;
    bool right_ok = right_max > pt_x;
    return left_ok && right_ok;
  }

  inline bool IsSplitBar(ObjectFullData::Ptr ref, ObjectFullData::Ptr &comp) {
    float ratio_diff =
        abs(comp->GetRatio() - ref->GetRatio()) / ref->GetRatio();
    float y_diff =
        abs(comp->GetCenter().y - ref->GetCenter().y) / ref->GetCenter().y;

    bool ratio_ok = ratio_diff < 0.1;
    bool y_diff_ok = y_diff < 0.1;
    return ratio_ok && y_diff_ok;
  }

  BooleanParameter _enable, _debug_contour, _search_only_bottom;
  // tbca = To Be Consider As
  IntegerParameter _min_length, _max_distance_from_bottom_bar_extremum,
      _min_area, _max_diff_from_90_tbca_horizontal,
      _max_diff_from_0_tbca_vertical, _min_percent_filled;

  cv::Mat _output_image;
  ObjectFeatureFactory _feat_factory;
};

}  // namespace lib_vision

#endif  // VISION_FILTER_FENCE_DETECTOR_H_
