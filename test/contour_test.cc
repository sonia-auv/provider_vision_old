/**
 * \file  contour_test.cc
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_atlas/config.h>
#include <lib_vision/algorithm/contour_list.h>

TEST(ContourTest, AllTest) {
  std::string path = atlas::kWorkspaceRoot +
      std::string("src/lib_vision/test/contour_test_img.png");

  cv::Mat img_tmp, img;

  img_tmp = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);

  img_tmp.convertTo(img, CV_8U);

  ASSERT_TRUE(img.type() == CV_8UC1);

//  img = cv::Mat::zeros(100,100,CV_8UC1);
  // Expect 7 contours

  ContourList contour_list_all(img, ContourList::METHOD::ALL);
  ASSERT_TRUE(contour_list_all._contour_list.size() == 7);

  // Expect 7 contours, hierarchy should be filled and same size.
  ContourList contour_list_hier(img, ContourList::METHOD::HIERARCHY);
  ASSERT_TRUE(contour_list_hier._contour_list.size() == 7);
  ASSERT_TRUE(contour_list_hier._hierarchy.size() == 7);

  // Expect 4
  ContourList contour_list_inner(img, ContourList::METHOD::INNER);
  ASSERT_TRUE(contour_list_inner._contour_list.size() == 4);

  // Expect 3
  ContourList contour_list_inner_most(img, ContourList::METHOD::INNER_MOST);
  ASSERT_TRUE(contour_list_inner_most._contour_list.size() == 3);

  // Expect 3
  ContourList contour_list_outer(img, ContourList::METHOD::OUTER);
  ASSERT_TRUE(contour_list_outer._contour_list.size() == 3);

  // Expect 1
  ContourList contour_list_outer_no_child(img, ContourList::METHOD::OUTER_NO_CHILD);
  ASSERT_TRUE(contour_list_outer_no_child._contour_list.size() == 1);

  // Since perfect square, should not approximate.
  Contour tmp (contour_list_outer_no_child[0]);
  tmp.Approximate(2);

  ASSERT_TRUE(tmp.size() == 4);

  std::vector<cv::Point>  vec;
  vec.push_back(cv::Point(0,0));
  vec.push_back(cv::Point(0,10));
  vec.push_back(cv::Point(1,11));
  vec.push_back(cv::Point(10,10));
  vec.push_back(cv::Point(10,0));
  Contour ctr(vec), ctr2(vec);
  ASSERT_TRUE( ctr._contour.size() == 5 );
  ctr.Approximate(3);
  ctr2.ApproximateBySize();
  ASSERT_TRUE( ctr._contour.size() == 4 );
  ASSERT_TRUE( ctr2._contour.size() == 4 );
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
