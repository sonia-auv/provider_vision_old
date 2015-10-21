/**
 * \file	ImageFile.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_IMAGE_H_
#define PROVIDER_VISION_MEDIA_IMAGE_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "provider_vision/utils/config.h"
#include "provider_vision/media/camera/base_media.h"

namespace vision_server {

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

  explicit ImageFile(const std::string &path_to_file) noexcept;

  virtual ~ImageFile() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /** Method override from Media */
  void Start() override;

  /** Method override from Media */
  void Stop() override;

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
  void NextImageCopy(cv::Mat &image) noexcept override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string path_;

  cv::Mat image_;
};

}  // namespace vision_server

#endif  // PROVIDER_VISION_MEDIA_IMAGE_H_
