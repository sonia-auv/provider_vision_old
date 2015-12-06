/**
 * \file	image_accumulator_buffer.cc
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

#include <lib_vision/algorithm/image_accumulator_buffer.h>

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ImageAccumulatorBuffer::ImageAccumulatorBuffer(int bufferLength,
                                               cv::Size imgSize, int type,
                                               METHOD method)
    : _buffer_size(bufferLength),
      _individual_weight(0.0f),
      _buffer_current_index(0),
      _image_vec(0),
      _image_type(type),
      _image_size(imgSize),
      _average_method(nullptr) {
  // Start with a buffer filled with blank matrices.
  FillWithBlank();
  _individual_weight = 1.0 / static_cast<float>(bufferLength);
  SetAverageMethod(method);
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::GetImage(cv::Mat &image) {
  // We do not want to access blank...&
  if (_buffer_size == 0) return;
  // Use the function in memory to average the accumulator and
  // reset the values
  (this->*_average_method)(image);
  image.convertTo(image, _image_type);
}

//------------------------------------------------------------------------------
//
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

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::ResetBuffer() { FillWithBlank(); }

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//
void ImageAccumulatorBuffer::AverageIncrease50Percent(cv::Mat &resultImage) {
  resultImage = _image_vec[GetIndexFromOldest(0)].clone();
  for (int i = 1; i < _buffer_size; i++) {
    cv::addWeighted(resultImage, 0.5, _image_vec[GetIndexFromOldest(i)], 0.5, 0,
                    resultImage);
  }
}

//------------------------------------------------------------------------------
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
