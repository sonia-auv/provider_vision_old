#include "contour.h"

//=============================================================================
//
//-----------------------------------------------------------------------------
//
Contour::Contour(const std::vector<cv::Point> ctr)
: _contour(ctr)
{
}

//-----------------------------------------------------------------------------
//
Contour::Contour(const cv::RotatedRect &rect)
{
  cv::Point2f pts[4];
  rect.points(pts);

  for (int j = 0; j < 4; j++) {
    _contour.push_back(pts[j]);
  }
}
