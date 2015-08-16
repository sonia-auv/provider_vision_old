/**
 * \file	ObjectFeatureCalculator.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_OBJECT_CALCULATOR_H_
#define VISION_FILTER_OBJECT_CALCULATOR_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <lib_vision/algorithm/features.h>
#include <lib_vision/algorithm/general_function.h>
#include <lib_vision/algorithm/target.h>
#include <lib_vision/algorithm/object_full_data.h>
#include <lib_vision/algorithm/feature_factory.h>
#include <lib_vision/algorithm/time.h>
#include <lib_vision/algorithm/ai_trainer.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class ObjectFeatureCalculator : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ObjectFeatureCalculator(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _recording_frame_index(0),
        _enable("Enable", false, parameters_),
        _debug_contour("Debug_contour", false, parameters_),
        _toggle_recording("toggle_recording", false, parameters_),
        _id("ID", "buoy", parameters_),
        _spec_1("spec1", "red", parameters_),
        _spec_2("spec2", "blue", parameters_),
        _output_folder("output_folder", "/home/jeremie/aidata/rec1/",
                       parameters_),
        _min_area("Min_area", 200, 0, 10000, parameters_),
        _feature_factory(5) {
    setName("ObjectFeatureCalculator");
    _feature_factory.SetAllFeatureToCompute();
    // Little goodies for cvs
    // area_rank,length_rank,circularity,convexity,ratio,presence,percent_filled,hueMean,
  }

  virtual ~ObjectFeatureCalculator() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      if (_debug_contour()) {
        image.copyTo(_output_image);
        if (_output_image.channels() == 1) {
          cv::cvtColor(_output_image, _output_image, CV_GRAY2BGR);
        }
      }

      if (image.channels() != 1) cv::cvtColor(image, image, CV_BGR2GRAY);
      cv::Mat originalImage = global_params_.getOriginalImage();

      Time timer;
      timer.UpdateStartTime();

      contourList_t contours;
      retrieveAllContours(image, contours);
      ObjectFullData::FullObjectPtrVec objVec;
      for (int i = 0, size = contours.size(); i < size; i++) {
        std::shared_ptr<ObjectFullData> object =
            std::make_shared<ObjectFullData>(originalImage, image, contours[i]);
        if (object.get() == nullptr) {
          continue;
        }

        if (object->GetArea() < _min_area()) {
          continue;
        }
        if (_debug_contour()) {
          cv::drawContours(_output_image, contours, i, CV_RGB(255, 0, 0), 2);
        }

        if (IsRectangle(contours[i], 10)) {
          cv::drawContours(_output_image, contours, i, CV_RGB(0, 255, 0), 2);

          objVec.push_back(object);
        }
      }
      // Calculate features
      //			_feature_factory.CalculateFeatureVectors(objVec);
      //			printf("Image parsing took: %f seconds\n",
      // timer.GetExecTime());
      if (_toggle_recording()) {
        AITrainer::OutputFrameData(_output_folder(), objVec, originalImage,
                                   image, _recording_frame_index);
        _recording_frame_index++;
      }
      if (_debug_contour()) {
        _output_image.copyTo(image);
      }
    }
  }

 private:
  cv::Mat _output_image;
  unsigned int _recording_frame_index;
  // Params
  BooleanParameter _enable, _debug_contour, _toggle_recording;
  StringParameter _id, _spec_1, _spec_2, _output_folder;
  DoubleParameter _min_area;

  FeatureFactory _feature_factory;
};

#define VISION_FILTER_PI 3.14159265

//==============================================================================
// U N I T   T E S T S

// We need to use another unit test framework such as google test that
// is being included in ROS framework.
// Commenting this test in the mean time so we can remove vitals dependencies.

//#include <TCTestHolder.h>
//
// TC_DEFINE_UNIT_TEST(ATAN_UT) {
//  cv::Mat in = cv::imread("/home/jeremie/Videos/AtanTest.png");
//  cv::Mat inGray;
//  cv::cvtColor(in, inGray, CV_BGR2GRAY);
//  contourList_t contours;
//      retrieveOuterContours(inGray, contours);
//  //	std::cout << "Testing angles" << std::endl;
//  //	for( int i = 0; i < 360; i++)
//  //	{
//  //		float y = tan(  i * PI / 180.0f );
//  //		cv::Mat A ( 1, 3, CV_32FC1, cv::Scalar::all(0));
//  //		cv::Mat B (1, 3, CV_32FC1, cv::Scalar::all(0));
//  //
//  //		float* ptrA = A.ptr<float>(0);
//  //		float* ptrB = B.ptr<float>(0);
//  //		ptrA[0] = 1, ptrA[1] = 0, ptrA[2] = 0;
//  //		ptrB[0] = 1, ptrB[1] = y, ptrB[2] = 0;
//  //		cv::Mat C = A.cross(B);
//  //		float* ptrC = C.ptr<float>(0);
//  //		float ninetyNorm = (norm(C))/(norm(A)*norm(B));
//  //		//std::cout << i <<" Norm: " << norm(C) << " Is not convex: " <<
//  // bool(ptrC[2] > 0) << " Square angle index: "<< ninetyNorm << std::endl;
//  //		std::cout << i <<"," << ninetyNorm << std::endl;
//  //
//  //	}
//  //	std::cout << "Ending Testing angles" << std::endl;
//
//  for (int i = 0; i < contours.size(); i++) {
//    cv::circle(in, contours[i][0], 3, CV_RGB(255, 0, 0));
//    cv::circle(in, contours[i][1], 3, CV_RGB(0, 255, 0));
//    contour_t contour = contours[i];
//    printf("New contour\n");
//
//    if (IsRectangle(contour)) {
//      std::cout << " IS A SQUARE!!! " << std::endl;
//      cv::drawContours(in, contours, i, CV_RGB(0, 255, 0));
//    }
//  }
//  cv::imshow("TIN", in);
//  cv::waitKey(-1);
//
//  return true;
//}
// TC_END_UNIT_TEST(ATAN_UT)

}  // namespace vision_filter

#endif  // VISION_FILTER_OBJECT_CALCULATOR_H_
