/**
 * \file	major_edge_extractor_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/major_edge_extractor.h>
#include <lib_vision/algorithm/performance_evaluator.h>

TEST(major_edge_extractor_test, AllTest) {
//  printf("Starting unit test on MajorEdgeExtractor");
//  cv::Mat original = cv::imread(
//      "/home/jeremie/sonia_log/vision_filter/library/doc/ORIGINAL_KBCE.png",
//      CV_LOAD_IMAGE_GRAYSCALE);
//  cv::Mat final = cv::imread(
//      "/home/jeremie/sonia_log/vision_filter/library/doc/FINAL_KBCE.png",
//      CV_LOAD_IMAGE_GRAYSCALE);
//
//  ASSERT_TRUE(!original.empty() && !final.empty());
//
//  MajorEdgeExtractor mee;
//
//  cv::Mat test_image(5, 5, CV_8UC1, cv::Scalar::all(0));
//  test_image.at<uchar>(2, 2) = 100;
//  mee.Init(cv::Size(5, 5));
//  // West
//  mee.AddRef(1, 2, 60);
//  // North
//  mee.AddRef(2, 1, 30);
//
//  for (int y = 1; y < 4; y++) {
//    for (int x = 1; x < 4; x++) {
//      std::cout << "X Y: " << x << " " << y << std::endl;
//      RefKernel kernel(mee._ref_image.at<RefPointPtr>(y - 1, x),
//                       mee._ref_image.at<RefPointPtr>(y, x - 1),
//                       mee._ref_image.at<RefPointPtr>(y, x));
//      uchar pix = " "
//                  << mee.IsWestExist(kernel) << " " << mee.IsAloneRef(kernel)
//                  << " " << mee.IsBothNortAndWestExist(kernel) << std::endl;
//
//      if (mee.IsNorthExist(kernel)) {
//        std::cout << "North Val Connected, greater: "
//                  << mee.IsValueConnected(kernel._north, pix) << "
//                                                                 "
//                  << mee.IsValueGreater(kernel._north, pix) << std::endl;
//      }
//
//      if (mee.IsWestExist(kernel)) {
//        std::cout << "West Val Connected, greater: "
//                  << mee.IsValueConnected(kernel._west, pix) << "
//                                                                "
//                  << mee.IsValueGreater(kernel._west, pix) << std::endl;
//      }
//
//      if (mee.IsBothNortAndWestExist(kernel)) {
//        std::cout << "both Val is junction " << mee.IsJunction(kernel, pix)
//                  << std::endl;
//        if (mee.IsJunction(kernel, pix)) {
//          mee.SetJunction(kernel, pix, x, y);
//        }
//      }
//    }
//  }
//
//  for (int y = 0; y < 5; y++) {
//    for (int x = 0; x < 5; x++) {
//      std::cout << mee._ref_image.at<RefPointPtr>(y, x) << "\t";
//    }
//    std::cout << std::endl;
//  }
//  for (int y = 0; y < 5; y++) {
//    for (int x = 0; x < 5; x++) {
//      if (mee._ref_image.at<RefPointPtr>(y, x) != nullptr)
//        std::cout << mee.GetValInReferenceVec(
//                         mee._ref_image.at<RefPointPtr>(y, x)) << "\t";
//    }
//    std::cout << std::endl;
//  }
//
//  cv::Mat float_ori;
//  original.convertTo(float_ori, CV_32F);
//  PerformanceEvaluator time;
//  time.UpdateStartTime();
//  cv::Mat algo_final = mee.ExtractEdge(float_ori, 50);
//  std::cout << "process: " << time.GetExecTimeSec() << std::endl;
//  cv::convertScaleAbs(algo_final, algo_final);
//
//  cv::imshow("Res", algo_final);
//
//  cv::imshow("Final", final);
//  cv::imshow("ori", original);
//
//  cv::waitKey(-1);
//  cv::Mat result;
//  cv::subtract(algo_final, final, result);
//  int test = cv::countNonZero(result);
//  std::cout << "TETS " << test << std::endl;
//  ASSERT_TRUE(test == 0);
//
//  printf("System all clear and good to go");
//  //    test_image.at<uchar>(y,x);
//  //			std::cout << "N W Alone Both: " <<
//  // mee.IsNorthExist(kernel)
//  //<<"
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
