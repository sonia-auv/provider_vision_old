/**
 * \file	contour_list.cc
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

#include "contour_list.h"

//=============================================================================
//  CONSTRUCTOR
ContourList::ContourList(const cv::Mat &image, const METHOD method) {
  switch (method) {
    default:
    case ALL:
      retrieveAllContours(image);
      break;
    case INNER:
      retrieveInnerContours(image);
      break;
    case INNER_MOST:
      retrieveInnerMostContours(image);
      break;
    case OUTER:
      retrieveOuterContours(image);
      break;
    case OUTER_NO_CHILD:
      retrieveOutNoChildContours(image);
      break;
    case HIERARCHY:
      retrieveHiearchyContours(image);
      break;
  }
  for( const auto &ctr : contour_list_point_)
  {
    contour_vec_.push_back(ctr);
  }
}

//=============================================================================
//  METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
void ContourList::retrieveAllContours(const cv::Mat &image) {
  // Clone because find contour modifies the image.
  cv::findContours(image.clone(), contour_list_point_, CV_RETR_LIST,
                   CV_CHAIN_APPROX_SIMPLE);
}

//-----------------------------------------------------------------------------
//
void ContourList::retrieveHiearchyContours(const cv::Mat &image) {
  cv::findContours(image.clone(), contour_list_point_, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);
}

//-----------------------------------------------------------------------------
//
void ContourList::retrieveInnerContours(const cv::Mat &image) {
  cv::findContours(image.clone(), contour_list_point_, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for (size_t i = 0, size = _hierarchy.size(); i < size; i++) {
    if (HasParent(_hierarchy[i])) {
      new_contour_list.push_back((contour_list_point_)[i]);
    }
  }

  std::swap(contour_list_point_, new_contour_list);
}

//-----------------------------------------------------------------------------
//
void ContourList::retrieveInnerMostContours(const cv::Mat &image) {
  cv::findContours(image.clone(), contour_list_point_, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for (size_t i = 0, size = _hierarchy.size(); i < size; i++) {
    if (HasParent(_hierarchy[i]) && !HasChild(_hierarchy[i])) {
      new_contour_list.push_back((contour_list_point_)[i]);
    }
  }

  std::swap(contour_list_point_, new_contour_list);
}

//-----------------------------------------------------------------------------
//
void ContourList::retrieveOuterContours(const cv::Mat &image) {
  cv::findContours(image.clone(), contour_list_point_, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for (size_t i = 0, size = _hierarchy.size(); i < size; i++) {
    if (!HasParent(_hierarchy[i])) {
      new_contour_list.push_back((contour_list_point_)[i]);
    }
  }

  std::swap(contour_list_point_, new_contour_list);
}

//-----------------------------------------------------------------------------
//
void ContourList::retrieveOutNoChildContours(const cv::Mat &image) {
  cv::findContours(image.clone(), contour_list_point_, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for (size_t i = 0, size = _hierarchy.size(); i < size; i++) {
    if (!HasParent(_hierarchy[i]) && !HasChild(_hierarchy[i])) {
      new_contour_list.push_back((contour_list_point_)[i]);
    }
  }

  std::swap(contour_list_point_, new_contour_list);
}
