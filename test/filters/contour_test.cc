/**
 * \file	contour_test.cc
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

#include <gtest/gtest.h>
#include <lib_atlas/config.h>
#include <provider_vision/algorithm/contour_list.h>

using namespace provider_vision;

TEST(ContourTest, AllTest) {
  std::string path = atlas::kWorkspaceRoot +
      std::string("src/provider_vision/test/img/contour_test_img.png");

  cv::Mat img_tmp, img;

  img_tmp = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);

  img_tmp.convertTo(img, CV_8U);

  ASSERT_TRUE(img.type() == CV_8UC1);

//  img = cv::Mat::zeros(100,100,CV_8UC1);
  // Expect 7 contours

  ContourList contour_list_all(img, ContourList::METHOD::ALL);
  ASSERT_TRUE(contour_list_all.GetSize() == 7);

  // Expect 7 contours, hierarchy should be filled and same size.
  ContourList contour_list_hier(img, ContourList::METHOD::HIERARCHY);
  ASSERT_TRUE(contour_list_hier.GetSize() == 7);
  ASSERT_TRUE(contour_list_hier.hierarchy_.size() == 7);

  // Expect 4
  ContourList contour_list_inner(img, ContourList::METHOD::INNER);
  ASSERT_TRUE(contour_list_inner.GetSize() == 4);

  // Expect 3
  ContourList contour_list_inner_most(img, ContourList::METHOD::INNER_MOST);
  ASSERT_TRUE(contour_list_inner_most.GetSize() == 3);

  // Expect 3
  ContourList contour_list_outer(img, ContourList::METHOD::OUTER);
  ASSERT_TRUE(contour_list_outer.GetSize() == 3);

  // Expect 1
  ContourList contour_list_outer_no_child(img, ContourList::METHOD::OUTER_NO_CHILD);
  ASSERT_TRUE(contour_list_outer_no_child.GetSize() == 1);

  // Since perfect square, should not approximate.
  Contour tmp (contour_list_outer_no_child[0]);
  tmp.Approximate(2);

  ASSERT_TRUE(tmp.GetSize() == 4);

  std::vector<cv::Point>  vec;
  vec.push_back(cv::Point(0,0));
  vec.push_back(cv::Point(0,10));
  vec.push_back(cv::Point(1,11));
  vec.push_back(cv::Point(10,10));
  vec.push_back(cv::Point(10,0));
  Contour ctr(vec), ctr2(vec);
  ASSERT_TRUE( ctr.contour_.size() == 5 );
  ctr.Approximate(3);
  ctr2.ApproximateBySize();
  ASSERT_TRUE( ctr.contour_.size() == 4 );
  ASSERT_TRUE( ctr2.contour_.size() == 4 );
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
