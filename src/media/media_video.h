/**
 * \file	MMVideo.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_MEDIA_VIDEO_H_
#define VISION_SERVER_MEDIA_VIDEO_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "config.h"
#include "media/media.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Handles image from files (png, jpeg) and is use as a camera
 * (same call for open, get image, close (start stop does nothing)
 */
class MMVideo : public Media, private cv::VideoCapture {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  MMVideo(std::string path_to_file, bool looping = true);

  MMVideo();

  virtual ~MMVideo();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void SetPathToVideo(std::string full_path);

  void SetLooping(bool looping);

  bool LoadVideo(std::string path_to_file);

  // Media overload
  std::vector<std::string> getCommands() const override;

  bool Start() override;

  bool Stop() override;

  bool NextImage(cv::Mat &image) override;

  bool IsRealCamera() const override;

  std::string GetName() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat _currentImage;

  std::string _path;

  bool _looping;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool MMVideo::IsRealCamera() const { return false; }

}  // namespace vision_server

#endif  // VISION_SERVER_MEDIA_VIDEO_H_
