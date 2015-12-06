/**
 * \file	white_noise_takedown.h
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

#ifndef LIB_VISION_FILTERS_WHITE_NOISE_TAKEDOWN_H_
#define LIB_VISION_FILTERS_WHITE_NOISE_TAKEDOWN_H_

#include <memory>
#include <lib_vision/filter.h>

namespace lib_vision {

class WhiteNoiseTakedown : public Filter {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<WhiteNoiseTakedown>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit WhiteNoiseTakedown(const GlobalParamHandler &globalParams)
      : Filter(globalParams),
        _enable("Enable", false, &parameters_),
        _lowB("LowB", 0, 0, 255, &parameters_),
        _highB("HighB", 0, 0, 255, &parameters_),
        _lowG("LowG", 0, 0, 255, &parameters_),
        _highG("HighG", 0, 0, 255, &parameters_),
        _lowR("LowR", 0, 0, 255, &parameters_),
        _highR("HighR", 0, 0, 255, &parameters_),
        _view_channel("Channel_view", 0, 0, 3, &parameters_,
                      "0=ALL, 1=Blue, 2=Green, 3=Red") {
    SetName("WhiteNoiseTakedown");
  }

  virtual ~WhiteNoiseTakedown() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual void Execute(cv::Mat &image) {
    if (_enable()) {
      std::vector<cv::Mat> channels;
      cv::Mat original_image(global_params_.getOriginalImage());
      cv::split(original_image, channels);
      cv::inRange(channels[0], _lowB(), _highB(), channels[0]);
      cv::inRange(channels[1], _lowG(), _highG(), channels[1]);
      cv::inRange(channels[2], _lowR(), _highR(), channels[2]);
      cv::Mat result;
      cv::bitwise_or(channels[0], channels[1], result);
      cv::bitwise_or(channels[2], result, result);
      std::vector<cv::Mat> res;

      switch (_view_channel()) {
        case 0:
          if (image.channels() == 3) {
            res.push_back(result);
            res.push_back(result);
            res.push_back(result);
            cv::merge(res, result);
            cv::bitwise_and(image, result, image);
          } else {
            cv::bitwise_and(image, result, image);
          }

          break;
        case 1:
          channels[0].copyTo(image);
          break;
        case 2:
          channels[1].copyTo(image);
          break;
        case 3:
          channels[2].copyTo(image);
          break;
      }
    }
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  Parameter<bool> _enable;
  RangedParameter<int> _lowB, _highB, _lowG, _highG, _lowR, _highR;
  RangedParameter<int> _view_channel;
};

}  // namespace lib_vision

#endif  // LIB_VISION_FILTERS_INRANGE_H_
