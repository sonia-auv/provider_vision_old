/*
 * contour_finder.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: jeremie
 */


#include "contour_list.h"

//=============================================================================
//  CONSTRUCTOR
ContourList::ContourList (const cv::Mat &image, const METHOD method)
{
  // To change for an exception
  //if (image.channels() != 1)
  //{
  //}

  switch (method)
  {
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
}

//=============================================================================
//  METHOD CODE SECTION
//-----------------------------------------------------------------------------
//
void
ContourList::retrieveAllContours(const cv::Mat &image)
{
  // Clone because find contour modifies the image.
  cv::findContours(image.clone(), _contour_list, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
}

//-----------------------------------------------------------------------------
//
void
ContourList::retrieveHiearchyContours(const cv::Mat &image)
{
   cv::findContours(image.clone(), _contour_list, _hierarchy, CV_RETR_TREE,
                    CV_CHAIN_APPROX_SIMPLE);
}

//-----------------------------------------------------------------------------
//
void
ContourList::retrieveInnerContours(const cv::Mat &image)
{
  cv::findContours(image.clone(), _contour_list, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for( size_t i = 0, size = _hierarchy.size(); i < size; i++)
  {
    if( HasParent(_hierarchy[i] ) )
    {
      new_contour_list.push_back( (_contour_list)[i] );
    }
  }

  std::swap(_contour_list, new_contour_list);
}

//-----------------------------------------------------------------------------
//
void
ContourList::retrieveInnerMostContours(const cv::Mat &image)
{
  cv::findContours(image.clone(), _contour_list, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for( size_t i = 0, size = _hierarchy.size(); i < size; i++)
  {
    if( HasParent( _hierarchy[i] ) && !HasChild(_hierarchy[i]))
    {
      new_contour_list.push_back( (_contour_list)[i] );
    }
  }

  std::swap(_contour_list, new_contour_list);
}

//-----------------------------------------------------------------------------
//
void
ContourList::retrieveOuterContours(const cv::Mat &image)
{
  cv::findContours(image.clone(), _contour_list, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for( size_t i = 0, size = _hierarchy.size(); i < size; i++)
  {
    if( !HasParent(_hierarchy[i]) )
    {
      new_contour_list.push_back( (_contour_list)[i] );
    }
  }

  std::swap(_contour_list, new_contour_list);
}

//-----------------------------------------------------------------------------
//
void
ContourList::retrieveOutNoChildContours(const cv::Mat &image)
{
  cv::findContours(image.clone(), _contour_list, _hierarchy, CV_RETR_TREE,
                   CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point> > new_contour_list;

  for( size_t i = 0, size = _hierarchy.size(); i < size; i++)
  {
    if( !HasParent(_hierarchy[i]) && !HasChild(_hierarchy[i]) )
    {
      new_contour_list.push_back( (_contour_list)[i] );
    }
  }

  std::swap(_contour_list, new_contour_list);
}
