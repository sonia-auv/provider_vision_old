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

#ifndef PROVIDER_VISION_MEDIA_CAMERA_VIDEO_FILE_H_
#define PROVIDER_VISION_MEDIA_CAMERA_VIDEO_FILE_H_

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"

namespace provider_vision {

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

  virtual ~VideoFile();

  //==========================================================================
  // P U B L I C   M E T H O D S

  bool Open() override;

  bool Close() override;

  // BaseMedia override
  bool SetStreamingModeOn() override;

  bool SetStreamingModeOff() override;

  bool NextImage(cv::Mat &image) override;

  void SetPathToVideo(const std::string &full_path);

  void SetLooping(bool looping);

  bool LoadVideo(const std::string &path_to_file);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Mat current_image_;

  std::string path_;

  bool looping_;
};
}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_VIDEO_FILE_H_
