/**
 * \file	general_function.h
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

#ifndef LIB_VISION_ALGORITHM_GENERAL_FUNCTION_H_
#define LIB_VISION_ALGORITHM_GENERAL_FUNCTION_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include <lib_vision/algorithm/type_and_const.h>
#include <lib_vision/algorithm/rot_rect.h>

#define ACCURACY_TABLE_SIZE 21
// Degree accuracy are the values corresponding to norm(A X B)/(norm(A) *
// norm(B))
// for angles of 90 +- index_in_array.
// Exemple:
// DEGREE_ACCURACY_FOR_90[5] is the result of norm(A X B)/(norm(A) * norm(B))
// when the angle between A and B is 90+t or 90-5
const float ACCURACY_TABLE[ACCURACY_TABLE_SIZE] = {
    1.0f,     0.999848, 0.999391, 0.99863,  0.997564, 0.996195, 0.994522,
    0.992546, 0.990268, 0.987688, 0.984808, 0.981627, 0.978148, 0.97437,
    0.970296, 0.965926, 0.961262, 0.956305, 0.951057, 0.945519, 0.939693};

// General function not associate to any object, that can be use by them
// or in the filters.

// Image splitting
// Return vector with blue,green,red,hue,saturation,intensity,gray (in order)
std::vector<cv::Mat> getColorPlanes(cv::Mat image);

// Camera offset
void setCameraOffset(cv::Point *pt, int rows, int cols);

// Contours getter
void retrieveContours(cv::Mat image, contourList_t &contours);

void retrieveInnerContours(cv::Mat image, contourList_t &contours);

void retrieveAllInnerContours(cv::Mat image, contourList_t &contours);

void retrieveOuterContours(cv::Mat image, contourList_t &contours);

void retrieveAllContours(cv::Mat image, contourList_t &contours);

void retrieveHiearchyContours(cv::Mat image, contourList_t &contours,
                              hierachy_t &hierarchy);

void retrieveOutNoChildContours(cv::Mat image, contourList_t &contours);

void retrieveContourRotRect(cv::RotatedRect rect, contour_t &contour);

void retrieveContourRotRect(RotRect rect, contour_t &contour);

// Features calculation
float calculateRatio(float width, float height);

float calculateConvexityRatio(contour_t contour);

float calculateConvexHullArea(contour_t contour);

float calculateCircleIndex(float area, float perimeter);

float calculateCircleIndex(contour_t contour);

// Receive a binary input and calculates the number of white pixel over the
// total number of pixel in the upright rectangle
float calculatePourcentFilled(const cv::Mat &image, const cv::Rect &rectangle);

float calculatePourcentFilled(const cv::Mat &image,
                              const cv::RotatedRect &rectangle);

cv::Scalar calculateMeans(contour_t contour, cv::Mat image, bool middle = true);

bool IsRectangle(contour_t &contour, unsigned int degreeAccuracy = 5);
bool IsSquare(std::vector<cv::Point> &approx, double min_area, double angle,
              double ratio_min, double ratio_max);

inline bool SortVerticesLength(const std::pair<unsigned int, cv::Vec3f> &a,
                               const std::pair<unsigned int, cv::Vec3f> &b) {
  return norm(a.second) > norm(b.second);
}

inline bool SortVerticesIndex(const std::pair<unsigned int, cv::Vec3f> &a,
                              const std::pair<unsigned int, cv::Vec3f> &b) {
  return a.first < b.first;
}

// Utilities
inline float eucledianPointDistance(const cv::Point2f &pt1,
                                    const cv::Point2f &pt2) {
  return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
}

cv::Mat extractImageFromRect(cv::RotatedRect rect, cv::Mat image);

cv::Mat extractImageFromRect(contour_t rect, cv::Mat image);

// Uses the enum given in type_and_const.h to control the rotation
cv::Mat rotateImage(cv::Mat in, rotationType rotation, symmetryType symmetry);

// Inverse a single channel image.
void inverseImage(const cv::Mat &in, cv::Mat &out);

// Process PCA
cv::Point getEigenPos(std::vector<cv::Point> &pts);
std::vector<double> getEigenValues(std::vector<cv::Point> &pts);
std::vector<cv::Point2d> getEigenVectors(std::vector<cv::Point> &pts);
double angleBetweenThreePoints(cv::Point pt1, cv::Point pt2, cv::Point pt0);
void drawSquares(cv::Mat &image,
                 const std::vector<std::vector<cv::Point> > &squares);
bool compareYX(const cv::Point &p1, const cv::Point &p2);

#endif  // LIB_VISION_ALGORITHM_GENERAL_FUNCTION_H_
