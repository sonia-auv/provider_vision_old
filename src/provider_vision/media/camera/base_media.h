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

#ifndef PROVIDER_VISION_MEDIA_CAMERA_BASE_MEDIA_H_
#define PROVIDER_VISION_MEDIA_CAMERA_BASE_MEDIA_H_

#include <provider_vision/media/camera_configuration.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include "provider_vision/config.h"

namespace provider_vision {

/**
 * Base class for anything that can provide an image to the system
 * implement basic functionality that is called through the system.
 */
class BaseMedia {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseMedia>;

  enum class Status { OPEN, STREAMING, CLOSE, ERROR };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BaseMedia(const std::string &name)
      : status_(Status::CLOSE), name_(name) {}

  virtual ~BaseMedia() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * For a camera, you can open the camera (i.e create it in the system)
   * but not make it stream (i.e. the hardware do not capture light)
   * therefore open and close are there to "make the camera exist in the
   * system"
   */
  virtual void Open() = 0;

  virtual void Close() = 0;

  /**
   * Starts to get images
   */
  void StartStreaming();

  /**
   * Stop getting images
   */
  void StopStreaming();

  /**
   * Gives the most recent image
   */
  virtual void NextImage(cv::Mat &image) = 0;

  /**
   * Call the next image and make a deep copy of it.
   * Warning, this method is very costly, use it only if you want the media to
   * keep the original image.
   */
  virtual void NextImageCopy(cv::Mat &image);

  /**
   * Returns the current camera Status
   */
  virtual const Status &GetStatus() const;

  /**
   * Makes return true if it does not have a proper framerate
   * i.e. Images and video;
   */
  virtual bool HasArtificialFramerate() const;

  const std::string &GetName() const;

  bool IsOpened() const;

  bool IsClosed() const;

  bool IsStreaming() const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Starts to get images
   */
  virtual void SetStreamingModeOn() = 0;

  /**
   * Stop getting images
   */
  virtual void SetStreamingModeOff() = 0;

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  Status status_;

  std::string name_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void BaseMedia::NextImageCopy(cv::Mat &image) {
  cv::Mat tmp_image;
  NextImage(tmp_image);
  image = tmp_image.clone();
}

//------------------------------------------------------------------------------
//
inline const BaseMedia::Status &BaseMedia::GetStatus() const {
  return status_;
};

//------------------------------------------------------------------------------
//
inline bool BaseMedia::HasArtificialFramerate() const { return true; }

//------------------------------------------------------------------------------
//
inline const std::string &BaseMedia::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
inline bool BaseMedia::IsOpened() const { return Status::OPEN == status_; }

//------------------------------------------------------------------------------
//
inline bool BaseMedia::IsClosed() const { return Status::CLOSE == status_; }

//------------------------------------------------------------------------------
//
inline bool BaseMedia::IsStreaming() const {
  return Status::STREAMING == status_;
}

//------------------------------------------------------------------------------
//
inline void BaseMedia::StartStreaming() {
  if (GetStatus() == Status::OPEN) {
    SetStreamingModeOn();
  } else if (GetStatus() == Status::CLOSE) {
    Open();
    SetStreamingModeOn();
  } else if (GetStatus() == Status::STREAMING) {
    // throw std::logic_error("The media is already streaming");
  } else {
    throw std::runtime_error(
        "The media is on an unstable status. Cannot stream.");
  }
}

//------------------------------------------------------------------------------
//
inline void BaseMedia::StopStreaming() {
  if (GetStatus() == Status::STREAMING) {
    SetStreamingModeOff();
  } else if (GetStatus() == Status::CLOSE) {
    throw std::logic_error("The media is not opened, cannot stop stream.");
  } else if (GetStatus() == Status::OPEN) {
    throw std::logic_error("The media is not streaming.");
  } else {
    throw std::runtime_error(
        "The media is on an unstable status. Cannot stream.");
  }
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_CAMERA_BASE_MEDIA_H_
