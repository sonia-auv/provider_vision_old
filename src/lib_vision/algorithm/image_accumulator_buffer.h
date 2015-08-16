/**
 * \file	image_accumulator_buffer.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_ACCUMULATOR_H_
#define VISION_FILTER_ACCUMULATOR_H_

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>

// Simple circular buffer
// that returns the weighted sum of all images in his buffer.
class ImageAccumulatorBuffer {
 public:
  enum METHOD { ACC_ALL_SAME_WEIGHT, ACC_50_PERCENT, ACC_ADJUST_WEIGHT };

  // Creates a circular buffer of bufferLength,
  // Filled with images of imgSize, with type type (8cu1, etc.).
  // At construction they are filled by zero, and they
  // are replaced as the images are added
  // NO CHECK IS MADE ON IMGSIZE AND TYPE, MAKE SURE YOU GIVE
  // TO THE FUNCTION PARAMETERS THAT WORK
  ImageAccumulatorBuffer(int bufferLength, cv::Size imgSize, int type,
                         METHOD method = ACC_ALL_SAME_WEIGHT);

  ~ImageAccumulatorBuffer(){};

  void AddImage(const cv::Mat &image);

  void GetImage(cv::Mat &image);

  // No concurency access check on buffer is made,
  // Do not use in different thread.
  void ResetBuffer();

  // No check is made on bufferLength.
  // Resize the accumulator and reset it.
  void ResetBuffer(int bufferLength, cv::Size imgSize, int type);

  // Set the average method via the Enum.
  void SetAverageMethod(METHOD method);

  // Return the buffer length
  int GetBufferLength();

  // Usefull for for loop iteration. ZERO BASED
  // Return the index of the
  // element at elementNumber from the most recent.
  // if the newest is at idx 4 (on a array of size 5)
  // elementNumber == 0 will return 4, and
  // elementNumber == 1 will return 3, etc.
  int GetIndexFromMostRecent(int elementNumber);

  // Return the index of the
  // element at elementNumber from the oldest.
  // if the oldest is at idx 4 (on a array of size 5)
  // elementNumber == 0 will return 4, and
  // elementNumber == 1 will return 0, etc.
  int GetIndexFromOldest(int elementNumber);

  void GetImage(unsigned int index, cv::Mat &image);

 private:
  // Averaging methods
  // They all keep the image in CV_32FCN
  // All images in the buffer have the same weights
  void AverageAllSameWeight(cv::Mat &resultImage);

  // Newest image has the most importance.
  // for size of 4,Img1 is older image, Img4 is newest
  // result = Img1*0.50 + Img2* 0.50
  // result = result*0.50 + Img3 * 0.50
  // result = result*0.50 + Img4 * 0.50
  void AverageIncrease50Percent(cv::Mat &resultImage);

  // for size of 4,Img1 is older image, Img4 is newest
  // result = Img4*0.50 + Img3*0.25 + Img2*0.125 + Imag*0.0625
  void AverageAccumulateWithResultingWeight(cv::Mat &resultImage);

  // Fills the accumulator with blank images
  void FillWithBlank();

  // Buffer size
  int _buffer_size;
  double _individual_weight;
  uint _buffer_current_index;

  std::vector<cv::Mat> _image_vec;
  // Image type.
  int _image_type;
  cv::Size _image_size;

  // Pointer to the method to use for
  // averaging the frame
  void (ImageAccumulatorBuffer::*_average_method)(cv::Mat &);
};

//=============================================================================
// INLINE FUNCTION

//-----------------------------------------------------------------------------
//
inline int ImageAccumulatorBuffer::GetBufferLength() { return _buffer_size; }

//-----------------------------------------------------------------------------
//
inline void ImageAccumulatorBuffer::GetImage(unsigned int index,
                                             cv::Mat &image) {
  if (index < _buffer_size) _image_vec[index].copyTo(image);
}

//// Debug function to use with constant value image (i.e. al at 50)
// std::vector<double> GetMeanInRecentOrder()
//{
//    cv::Scalar mean;
//    std::vector<double> meanVals;
//    for(int i = 0; i < _buffer_size; i++)
//    {
//        mean = cv::mean(_image_vec[GetIndexFromMostRecent(i)]);
//        meanVals.push_back(mean[0]);
//    }
//    return meanVals;
//}
//
// std::vector<double> GetMeanInOldedOrder()
//{
//    cv::Scalar mean;
//    std::vector<double> meanVals;
//    for(int i = 0; i < _buffer_size; i++)
//    {
//        mean = cv::mean(_image_vec[GetIndexFromOldest(i)]);
//        meanVals.push_back(mean[0]);
//    }
//    return meanVals;
//}
// void testFunction{
//    ImageAccumulatorBuffer acc(5, cv::Size(50,50), CV_8UC1);
//
//    std::vector<cv::Mat> increasingTest, constantTest;
//    for(int i = 0; i < 5; i++)
//    {
//        constantTest.push_back(cv::Mat(cv::Size(100,100), CV_8UC3,
//        cv::Scalar(100)));
//        increasingTest.push_back(cv::Mat(cv::Size(50,50), CV_8UC1,
//        cv::Scalar((i+1)*50)));
//    }
//
//    for( int i = 0; i < 7; i++)
//    {
//        acc.AddImage(increasingTest[i%5]);
//    }
//    acc.ResetBuffer();
//    // Should fail, since the
//    for( int i = 0; i < 7; i++)
//    {
//        acc.AddImage(constantTest[i%5]);
//    }
//    acc.ResetBuffer(3,cv::Size(100,100), CV_8UC3);
//    for( int i = 0; i < 7; i++)
//    {
//        acc.AddImage(constantTest[i%5]);
//    }
//    acc.ResetBuffer( 5, cv::Size(50,50), CV_8UC1);
//    for( int i = 0; i < 7; i++)
//    {
//        acc.AddImage(increasingTest[i%5]);
//    }
//
//    for(int i = 0; i < acc.GetAccumulatorSize();i++)
//    {
//        printf("%d ", acc.GetIndexFromMostRecent(i));
//    }
//    printf("\n");
//
//    for(int i = 0; i < acc.GetAccumulatorSize();i++)
//    {
//
//        printf("%d ", acc.GetIndexFromOldest(i));
//    }
//    printf("\n");
//
//    std::vector<double> meanVec = acc.GetMeanInRecentOrder();
//    for(int i = 0; i < meanVec.size(); i ++)
//    {
//        std::cout<< meanVec[i] << "\t";
//    }
//    std::cout<< std::endl;
//
//    meanVec = acc.GetMeanInOldedOrder();
//    for(int i = 0; i < meanVec.size(); i ++)
//    {
//        std::cout<< meanVec[i] << "\t";
//    }
//    std::cout<< std::endl;
//
//
//    cv::Mat test;
//    // using ACC_ALL_SAME_WEIGHT (default in constructor)
//    acc.GetImage(test);
//    cv::Scalar mean = cv::mean(test);
//    //Return 150
//    std::cout << mean << std::endl;
//    acc.SetAverageMethod(ImageAccumulatorBuffer::ACC_50_PERCENT);
//    acc.GetImage(test);
//    mean = cv::mean(test);
//    //Return 134
//    std::cout << "50 Percent " << mean << std::endl;
//    acc.SetAverageMethod(ImageAccumulatorBuffer::ACC_ADJUST_WEIGHT);
//    acc.GetImage(test);
//    mean = cv::mean(test);
//    //Return 128
//    std::cout << "Adjust Weight" << mean << std::endl;
//}
#endif
