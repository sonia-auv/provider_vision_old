/**
 * \file	VideoFile.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_VIDEO_H_
#define PROVIDER_VISION_MEDIA_VIDEO_H_

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"

namespace vision_server {

/**
 * Handles image from files (png, jpeg) and is use as a camera
 * (same call for open, get image, close (start stop does nothing)
 */
class VideoFile : public BaseMedia, private cv::VideoCapture {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = VideoFile::Ptr;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit VideoFile(const std::string &path_to_file, bool looping = true);

  VideoFile();

  virtual ~VideoFile();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void SetPathToVideo(const std::string &full_path);

  void SetLooping(bool looping);

  bool LoadVideo(const std::string &path_to_file);

  // BaseMedia override
  void Start() override;

  void Stop() override;

  void NextImage(cv::Mat &image) override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat current_image_;

  std::string path_;

  bool looping_;
};
}  // namespace vision_server

#endif  // PROVIDER_VISION_MEDIA_VIDEO_H_
