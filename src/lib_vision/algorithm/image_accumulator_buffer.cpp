/**
 * \file	general_accumulator_buffer.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/image_accumulator_buffer.h>

//=============================================================================
//              CONSTRUCTOR
//=============================================================================
ImageAccumulatorBuffer::ImageAccumulatorBuffer(int bufferLength,
                                               cv::Size imgSize, int type,
                                               METHOD method)
    : _buffer_size(bufferLength),
      _individual_weight(0.0f),
      _buffer_current_index(0),
      _image_vec(0),
      _image_type(type),
      _image_size(imgSize),
      _average_method(NULL) {
  // Start with a buffer filled with blank matrices.
  FillWithBlank();
  _individual_weight = 1.0 / static_cast<float>(bufferLength);
  SetAverageMethod(method);
}

//=============================================================================
//
void ImageAccumulatorBuffer::GetImage(cv::Mat &image) {
  // We do not want to access blank...&
  if (_buffer_size == 0) return;
  // Use the function in memory to average the accumulator and
  // reset the values
  (this->*_average_method)(image);
  image.convertTo(image, _image_type);
}

//=============================================================================
//              Method section
void ImageAccumulatorBuffer::AddImage(const cv::Mat &image) {
  if (_image_size != image.size() || _image_type != image.type()) {
    printf(
        "ImageAccumulatorBuffer: Image type or image size is different from "
        "the "
        "one I was constructed with.\n");
    return;
  }

  // Put everything in float, so we don't have issues with decimals and
  // image type mismatch
  // CV_MAT_CN return the number of channel.
  int index = _buffer_current_index % _buffer_size;
  image.convertTo(_image_vec[index], CV_32FC(CV_MAT_CN(_image_type)));
  // Here, since unsigned value, will return to zero after "overflowing"
  _buffer_current_index++;
}

//=============================================================================
//
void ImageAccumulatorBuffer::ResetBuffer() { FillWithBlank(); }

//=============================================================================
//
void ImageAccumulatorBuffer::ResetBuffer(int bufferLength, cv::Size imgSize,
                                         int type) {
  _buffer_size = bufferLength;
  _image_size = imgSize;
  _image_type = type;
  _buffer_current_index = 0;
  _individual_weight = 1.0 / static_cast<float>(bufferLength);

  FillWithBlank();
}

//=============================================================================
//
void ImageAccumulatorBuffer::AverageAllSameWeight(cv::Mat &resultImage) {
  if (_buffer_size == 0) {
    printf("Image accumulator size is 0\n");
    return;
  }

  cv::Mat adderImage =
      cv::Mat::zeros(_image_size, CV_32FC(CV_MAT_CN(_image_type)));
  for (int i = 0; i < _buffer_size; i++) {
    cv::add(adderImage, _image_vec[i], adderImage);
  }
  // Divide and cast into the image type given in at construction or resizing.
  cv::divide(adderImage, cv::Mat(_image_size, CV_32FC(CV_MAT_CN(_image_type)),
                                 cv::Scalar::all(_buffer_size)),
             resultImage);
}

//=============================================================================
//
void ImageAccumulatorBuffer::AverageIncrease50Percent(cv::Mat &resultImage) {
  resultImage = _image_vec[GetIndexFromOldest(0)].clone();
  for (int i = 1; i < _buffer_size; i++) {
    cv::addWeighted(resultImage, 0.5, _image_vec[GetIndexFromOldest(i)], 0.5, 0,
                    resultImage);
  }
}

//=============================================================================
//
void ImageAccumulatorBuffer::AverageAccumulateWithResultingWeight(
    cv::Mat &resultImage) {
  // Fill resultImage
  // Keep in float to stay at full scale.
  resultImage = cv::Mat::zeros(_image_size, CV_32FC(CV_MAT_CN(_image_type)));
  cv::addWeighted(resultImage, 0.5, _image_vec[GetIndexFromMostRecent(0)], 0.5,
                  0, resultImage);

  float resultingWeight = 0.50f;
  // Start at one, we already did the 0 index (newest)
  for (int i = 1; i < _buffer_size; i++) {
    // Addjust the weight, we are one element older now then before,
    // so the weight of this element is half the precendent.
    resultingWeight = resultingWeight / 2;

    cv::add(resultImage,
            _image_vec[GetIndexFromMostRecent(i)] * resultingWeight,
            resultImage);
  }
}

