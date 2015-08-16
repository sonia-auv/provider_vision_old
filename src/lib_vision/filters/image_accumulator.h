/**
 * \file	image_accumulator.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	14/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_IMAGE_ACCUMULATOR_H_
#define VISION_FILTER_IMAGE_ACCUMULATOR_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <lib_vision/filter.h>
#include <lib_vision/algorithm/image_accumulator_buffer.h>
#include <lib_vision/algorithm/time.h>

namespace vision_filter {

//==============================================================================
// C L A S S E S

class ImageAccumulator : public Filter {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ImageAccumulator(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _accumulator(3, cv::Size(0, 0), CV_8UC1),
        _enable("Enable", false, parameters_),
        _nb_image("NB_of_images", 3, 1, 20, parameters_),
        _method("Method_to_use", 1, 0, 2, parameters_,
                "Method: 1=SameWeight, 2=Adding50Percent, 3=Adjusted"),
        _last_size(0, 0),
        _last_method(CV_8UC1),
        _last_type(0),
        _last_nb_image(3) {
    setName("ImageAccumulator");
  }

  virtual ~ImageAccumulator() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void execute(cv::Mat &image) {
    if (_enable()) {
      // Is there any change in the type of images
      // we input to the accumulator?
      // If yes, reset it.
      if (_last_type != image.type() || _last_method != _method() ||
          _last_nb_image != _nb_image() || _last_size != image.size()) {
        _accumulator.ResetBuffer(_nb_image(), image.size(), image.type());

        _last_nb_image = _nb_image();
        _last_size = image.size();

        switch (_method()) {
          case 0:
            _accumulator.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_ALL_SAME_WEIGHT);
            break;
          case 1:
            _accumulator.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_50_PERCENT);
            break;
          case 2:
            _accumulator.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_ADJUST_WEIGHT);
            break;
          default:
            _accumulator.SetAverageMethod(
                ImageAccumulatorBuffer::ACC_ALL_SAME_WEIGHT);
            break;
        }
        _last_method = _method();
        _last_type = image.type();
      }
      // Add the newest frame
      _accumulator.AddImage(image);
      // Change the input for the newest averaging.
      _accumulator.GetImage(image);
    }
  }

 private:
  // Params
  ImageAccumulatorBuffer _accumulator;
  BooleanParameter _enable;
  IntegerParameter _nb_image, _method;
  // Here we need some sorte of remembering
  // so we can reset the accumulator on
  // param changing.
  cv::Size _last_size;
  int _last_method, _last_type, _last_nb_image;
};

}  // namespace vision_filter

#endif  // VISION_FILTER_IMAGE_ACCUMULATOR_H_
