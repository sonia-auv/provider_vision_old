/**
 * \file	MMImage.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_MEDIA_IMAGE_H_
#define VISION_SERVER_MEDIA_IMAGE_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utils/constants.h"
#include "media/media.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Handles image from files (png, jpeg) and is use s a camera
 * (same call for open, get image, close (start stop does nothing)
 */
class MMImage : public Media {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  MMImage(std::string path_to_file);

  MMImage();

  virtual ~MMImage(){};

  //==========================================================================
  // P U B L I C   M E T H O D S

  bool LoadImage(std::string path_to_file);

  /** Method override from Media */
  std::vector<std::string> getCommands() const override;

  /** Method override from Media */
  bool Start() override;

  /** Method override from Media */
  bool Stop() override;

  /** Method override from Media */
  bool NextImage(cv::Mat &image) override;

  bool IsRealCamera() const override;

  /** Method override from Media */
  std::string GetName() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat _image;

  std::string _path;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool MMImage::IsRealCamera() const { return false; }

}  // namespace vision_server

#endif  // VISION_SERVER_MEDIA_IMAGE_H_
