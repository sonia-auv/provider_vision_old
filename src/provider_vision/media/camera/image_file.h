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
#include "provider_vision/config.h"
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

  ImageFile(std::string path_to_file);

  ImageFile();

  virtual ~ImageFile(){};

  //==========================================================================
  // P U B L I C   M E T H O D S

  bool LoadImage(std::string path_to_file);

  /** Method override from Media */
  bool Start() override;

  /** Method override from Media */
  bool Stop() override;

  /** Method override from Media */
  bool NextImage(cv::Mat &image) override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat image_;

  std::string path_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

}  // namespace vision_server

#endif  // PROVIDER_VISION_MEDIA_IMAGE_H_
