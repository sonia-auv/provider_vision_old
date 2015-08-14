/**
 * \file	major_edge_extractor.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/major_edge_extractor.h>

//=============================================================================
//		CONSTANT
const float MajorEdgeExtractor::PERCENT_OF_VAL_FOR_VALUE_CONNECTION = 0.8;

//=============================================================================
//		CONTAINER CLASS
//-----------------------------------------------------------------------------
//
ReferencePoint::ReferencePoint(float pix_val, int max_val_index)
    : _pix_value(pix_val), _reference_max_index(max_val_index) {}

//-----------------------------------------------------------------------------
//
RefKernel::RefKernel(const RefPointPtr &north, const RefPointPtr &west,
                     const RefPointPtr &center)
    : _north(north), _west(west), _center(center) {}

//=============================================================================
//		MAIN CLASS
//-----------------------------------------------------------------------------
//
void MajorEdgeExtractor::Init(const cv::Size &size) {
  // If the image is already created, no need for
  // re-creating it.
  if (_ref_image.size() != size) CreateRefImage(size);
}

//-----------------------------------------------------------------------------
//
void MajorEdgeExtractor::Clean() {
  // Free the RefPoint allocated previously.
  _max_value_reference.clear();
  for (int y = 0, rows = _ref_image.rows, cols = _ref_image.cols; y < rows;
       y++) {
    RefPointPtr *ptr = _ref_image.ptr<RefPointPtr>(y);
    for (int x = 0; x < cols; x++) {
      if (ptr[x] != nullptr) {
        free(ptr[x]);
      }
    }
  }
}

//-----------------------------------------------------------------------------
//
void MajorEdgeExtractor::SetJunction(RefKernel &ref_kernel, float value, int x,
                                     int y) {
  if (ref_kernel._north == nullptr || ref_kernel._west == nullptr) return;

  RefPointPtr first_value = ref_kernel._north;
  RefPointPtr second_value = ref_kernel._west;

  if (GetValInReferenceVec(first_value) < GetValInReferenceVec(second_value)) {
    std::swap(first_value, second_value);
  }

  SetLink(first_value, value, x, y);

  SetValInReferenceVec(second_value, GetValInReferenceVec(first_value));
}

//-----------------------------------------------------------------------------
//
cv::Mat MajorEdgeExtractor::ExtractEdge(const cv::Mat &image,
                                        int extreme_minimum) {
  if (image.channels() != 1 || image.type() != CV_32F) {
    std::cout << "Bad image type or number of channel" << std::endl;
    return cv::Mat::zeros(1, 1, CV_8UC1);
  }

  // Image creation
  cv::Mat final_image(image.size(), CV_8UC1, 0);
  cv::Mat working_image;
  cv::copyMakeBorder(image, working_image, 1, 1, 1, 1, cv::BORDER_DEFAULT);
  Init(working_image.size());

  for (int y = 1, rows = working_image.rows, cols = working_image.cols;
       y < rows - 1; y++) {
    float *ptr = working_image.ptr<float>(y);
    RefPointPtr *ref_up_line = _ref_image.ptr<RefPointPtr>(y - 1);
    RefPointPtr *ref_center_line = _ref_image.ptr<RefPointPtr>(y);
    for (int x = 1; x < cols - 1; x++) {
      RefKernel ref_kernel(ref_up_line[x], ref_center_line[x - 1],
                           ref_center_line[x]);
      float pix_val = ptr[x];
      // Pixel is too low in value, does not workt being looked at...
      if (pix_val < extreme_minimum) {
        continue;
      }

      if (IsAloneRef(ref_kernel)) {
        AddRef(x, y, pix_val);
        continue;
      }

      if (IsBothNortAndWestExist(ref_kernel)) {
        if (IsJunction(ref_kernel, pix_val)) {
          SetJunction(ref_kernel, pix_val, x, y);
          continue;
        }
      }

      if (IsWestExist(ref_kernel)) {
        if (IsValueConnected(ref_kernel._west, pix_val)) {
          SetLink(ref_kernel._west, pix_val, x, y);
        }
      } else if (IsNorthExist(ref_kernel)) {
        if (IsValueConnected(ref_kernel._north, pix_val)) {
          SetLink(ref_kernel._north, pix_val, x, y);
        }
      }
    }
  }

  for (int y = 0, rows = final_image.rows, cols = final_image.cols; y < rows;
       y++) {
    float *val_ptr = final_image.ptr<float>(y);
    RefPointPtr *ref_ptr = _ref_image.ptr<RefPointPtr>(y + 1);

    for (int x = 0; x < cols; x++) {
      if (ref_ptr[x + 1] != nullptr) {
        // val_ptr[x] = GetValInReferenceVec(ref_ptr[x+1]);
        val_ptr[x] = 255;
      }
    }
  }

  Clean();

  return final_image;
}

//=============================================================================
//=============================================================================
//=============================================================================
//=========================== UNIT TEST AREA ==================================
//=============================================================================
//=============================================================================
//=============================================================================
#include <TCUnitTest.h>
#include <lib_vision/algorithm/time.h>

TC_DEFINE_UNIT_TEST(MajorEdgeExtractorUT) {
  printf("Starting unit test on MajorEdgeExtractor");
  cv::Mat original = cv::imread(
      "/home/jeremie/sonia_log/vision_filter/library/doc/ORIGINAL_KBCE.png",
      CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat final = cv::imread(
      "/home/jeremie/sonia_log/vision_filter/library/doc/FINAL_KBCE.png",
      CV_LOAD_IMAGE_GRAYSCALE);

  TC_TEST_FAIL("Images not loaded", !original.empty() && !final.empty());

  MajorEdgeExtractor mee;

  //	cv::Mat test_image(5,5,CV_8UC1, cv::Scalar::all(0));
  //	test_image.at<uchar>(2,2) = 100;
  //	mee.Init(cv::Size(5,5));
  //	// West
  //	mee.AddRef(1,2,60);
  //	// North
  //	mee.AddRef(2,1,30);
  //
  //	for( int y = 1; y < 4; y++)
  //	{
  //		for( int x = 1; x < 4; x++)
  //		{
  //			std::cout << "X Y: " << x << " " << y << std::endl;
  // 			RefKernel kernel(mee._ref_image.at<RefPointPtr>(y-1,x),
  //					mee._ref_image.at<RefPointPtr>(y,x-1),
  //					mee._ref_image.at<RefPointPtr>(y,x) );
  //			uchar pix = test_image.at<uchar>(y,x);
  //			std::cout << "N W Alone Both: " <<
  // mee.IsNorthExist(kernel)
  //<<"
  //"
  //					<< mee.IsWestExist(kernel) <<" "
  //					<< mee.IsAloneRef(kernel) <<" "
  //					<< mee.IsBothNortAndWestExist(kernel) <<
  // std::endl;
  //
  //			if( mee.IsNorthExist(kernel) )
  //			{
  //				std::cout << "North Val Connected, greater: " <<
  //						mee.IsValueConnected(kernel._north,
  //pix)
  //<<
  //"
  //"
  //						<<
  // mee.IsValueGreater(kernel._north,pix)
  //<<
  // std::endl;
  //			}
  //
  //			if( mee.IsWestExist(kernel) )
  //			{
  //				std::cout << "West Val Connected, greater: " <<
  //						mee.IsValueConnected(kernel._west,
  //pix)
  //<<
  //"
  //"
  //						<<
  // mee.IsValueGreater(kernel._west,pix)
  //<<
  // std::endl;
  //			}
  //
  //			if( mee.IsBothNortAndWestExist(kernel) )
  //			{
  //				std::cout << "both Val is junction " <<
  //						mee.IsJunction(kernel, pix) <<
  // std::endl;
  //				if( mee.IsJunction(kernel, pix) )
  //				{
  //					mee.SetJunction(kernel, pix, x, y);
  //				}
  //			}
  //		}
  //	}
  //
  //	for( int y = 0; y < 5; y++)
  //	{
  //		for( int x = 0; x < 5; x++)
  //		{
  //			std::cout << mee._ref_image.at<RefPointPtr>(y,x) <<
  //"\t";
  //		}
  //		std::cout << std::endl;
  //	}
  //	for( int y = 0; y < 5; y++)
  //	{
  //		for( int x = 0; x < 5; x++)
  //		{
  //			if( mee._ref_image.at<RefPointPtr>(y,x) != nullptr)
  //			std::cout <<
  // mee.GetValInReferenceVec(mee._ref_image.at<RefPointPtr>(y,x))<< "\t";
  //		}
  //		std::cout << std::endl;
  //	}

  cv::Mat float_ori;
  original.convertTo(float_ori, CV_32F);
  Time time;
  time.UpdateStartTime();
  cv::Mat algo_final = mee.ExtractEdge(float_ori, 50);
  std::cout << "process: " << time.GetExecTime() << std::endl;
  cv::convertScaleAbs(algo_final, algo_final);

  cv::imshow("Res", algo_final);

  cv::imshow("Final", final);
  cv::imshow("ori", original);

  cv::waitKey(-1);
  cv::Mat result;
  cv::subtract(algo_final, final, result);
  int test = cv::countNonZero(result);
  std::cout << "TETS " << test << std::endl;
  TC_TEST_FAIL("Images not the same", test == 0);

  printf("System all clear and good to go");
  return true;
}
TC_END_UNIT_TEST(MajorEdgeExtractorUT);
