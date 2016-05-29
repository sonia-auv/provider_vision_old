/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#ifndef PROVIDER_VISION_MEDIA_CAMERA_IMAGE_FILE_H_
#define PROVIDER_VISION_MEDIA_CAMERA_IMAGE_FILE_H_

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"

namespace provider_vision {

/**
 * Handles image from files (png, jpeg) and is use s a camera
 * (same call for open, get image, close (start stop does nothing)
 */
class ImageFile : public BaseMedia {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ImageFile>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ImageFile(const std::string &path_to_file);

  virtual ~ImageFile();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Open() override;

  void Close() override;

  /** Method override from Media */
  void SetStreamingModeOn() override;

  /** Method override from Media */
  void SetStreamingModeOff() override;

  /**
   * Method override from Media.
   *
   * We are forced to clone the image here as the user could modify the image,
   * we do not want to loose the origin image neither to relaunch at each
   * NextImage() call
   */
  void NextImage(cv::Mat &image) override;

  /**
   * Get a deep copy of the image.
   *
   * We must override this method as we do not want the user to clone the
   * image twice by calling it.
   * We will simply call NextImage and return a swallow copy.
   */
  void NextImageCopy(cv::Mat &image) override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string path_;

  cv::Mat image_;
};

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_IMAGE_FILE_H_
