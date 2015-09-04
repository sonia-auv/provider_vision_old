/*
 * contour_finder.h
 *
 *  Created on: Aug 24, 2015
 *      Author: jeremie
 */

#ifndef LIB_VISION_SRC_LIB_VISION_ALGORITHM_CONTOUR_LIST_H_
#define LIB_VISION_SRC_LIB_VISION_ALGORITHM_CONTOUR_LIST_H_

#include <opencv2/opencv.hpp>

#include "contour.h"
// Contour finding


class ContourList
{
public:

  // Contour navigation constant
  static const unsigned int NEXT_CTR = 0;
  static const unsigned int PREV_CTR = 1;
  static const unsigned int FIRST_CHILD_CTR = 2;
  static const unsigned int PARENT_CTR = 3;

  enum METHOD {
    ALL, // All the contour in the image
    INNER, // Every contour with a parent
    INNER_MOST, // Every contour with a parent AND no child
    OUTER, // Every contour without a parent
    OUTER_NO_CHILD, // Ever contour without a parent AND no child
    HIERARCHY // All the contours, fills the hierarchy member
  };

  ContourList (const cv::Mat &image, const METHOD method);

  void DrawContours(cv::Mat &img, const cv::Scalar &color,
                    int thickness = 2);

  // Vector overload
  size_t size();
  std::vector<cv::Point> operator[](unsigned int index);

private:

  bool HasChild(const cv::Vec4i &hierarchy_def);
  bool HasParent(const cv::Vec4i &hierarchy_def);

  // Retrieval method
  void retrieveAllContours(const cv::Mat &image);

  // Retrieve contour with hierachy. Sets the vector _hierachy of this object.
  void retrieveHiearchyContours(const cv::Mat &image);

  // All innermost contour i.e. no child
  void retrieveInnerContours(const cv::Mat &image);

  // All inner contour i.e. has a parent
  void retrieveInnerMostContours(const cv::Mat &image);

  // All outer contour i.e. has no parent
  void retrieveOuterContours(const cv::Mat &image);

  // All contour that has no child AND no parent
  void retrieveOutNoChildContours(const cv::Mat &image);

public:

  std::vector<std::vector<cv::Point> > _contour_list;
  // Contains the hierachy when METHOD used is HIERACHY
  std::vector<cv::Vec4i> _hierarchy;
};

//=============================================================================
//
//-----------------------------------------------------------------------------
//
inline size_t
ContourList::size()
{
  return _contour_list.size();
}

//-----------------------------------------------------------------------------
//
inline std::vector<cv::Point>
ContourList::operator[](unsigned int index)
{
  return _contour_list[index];
}

//-----------------------------------------------------------------------------
//
inline bool ContourList::HasChild(const cv::Vec4i &hierarchy_def)
{
   return hierarchy_def[FIRST_CHILD_CTR] != -1;
}

//-----------------------------------------------------------------------------
//
inline bool ContourList::HasParent(const cv::Vec4i &hierarchy_def)
{
  return hierarchy_def[PARENT_CTR] != -1;
}

//-----------------------------------------------------------------------------
//
inline void ContourList::DrawContours(cv::Mat &img, const cv::Scalar &color,
                                      int thickness)
{
   cv::drawContours(img, _contour_list, -1, color, thickness);
}

#endif /* LIB_VISION_SRC_LIB_VISION_ALGORITHM_CONTOUR_FINDER_H_ */
