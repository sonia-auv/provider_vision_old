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

#include "provider_vision/media/camera/video_file.h"
#include <string>
#include <vector>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
VideoFile::VideoFile(const std::string &path_to_file, bool looping)
    : BaseMedia(path_to_file),
      VideoCapture(path_to_file),
      current_image_(),
      path_(path_to_file),
      looping_(looping) {
  LoadVideo(path_);
}

//------------------------------------------------------------------------------
//
VideoFile::~VideoFile() {
  if (isOpened()) {
    release();
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool VideoFile::Open() {
  // Might be already open since we do it on construction if the path is
  // provided
  if (!isOpened()) {
    LoadVideo(path_);
  }

  if (!isOpened()) {
    ROS_ERROR("The video %s could not be opened.", path_.c_str());
    return false;
  }

  status_ = Status::OPEN;
  return true;
}

//------------------------------------------------------------------------------
//
bool VideoFile::Close() {
  if (isOpened()) {
    release();
  }

  if (isOpened()) {
    ROS_ERROR("The video %s could not be closed.", path_.c_str());
    return false;
  }
  status_ = Status::CLOSE;
  return true;
}

//------------------------------------------------------------------------------
//
bool VideoFile::SetStreamingModeOn() {
  status_ = Status::STREAMING;
  return true;
}

//------------------------------------------------------------------------------
//
bool VideoFile::SetStreamingModeOff() {
  status_ = Status::CLOSE;
  return true;
}

//------------------------------------------------------------------------------
//
void VideoFile::SetLooping(bool looping) { looping_ = looping; }

//------------------------------------------------------------------------------
//
bool VideoFile::LoadVideo(const std::string &path_to_file) {
  return open(path_to_file);
}

//------------------------------------------------------------------------------
//
bool VideoFile::NextImage(cv::Mat &image) {
  if (isOpened()) {
    // Clear the previous image.
    current_image_ = cv::Mat();

    operator>>(current_image_);
    if (!current_image_.empty()) {
      // Next image has loaded
      image = current_image_;
    } else if (looping_) {
      // end of sequence, going back to first frame.
      // Delay will happen, but it's part of the lib...
      set(CV_CAP_PROP_POS_AVI_RATIO, 0);
    } else {
      ROS_ERROR("No image could be acquiered from this media %s.",
                path_.c_str());
      return false;
    }
    return true;
  }
  return false;
}

}  // namespace provider_vision
