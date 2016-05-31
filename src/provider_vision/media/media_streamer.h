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

#ifndef PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_
#define PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_

#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/sys/fsinfo.h>
#include <lib_atlas/sys/timer.h>
#include <ros/ros.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include "provider_vision/media/camera/base_media.h"

namespace provider_vision {

/**
 * Class responsible of acquiring an image from a device, asynchronously from
 * the system. It is basically a thread running and getting images from a media.
 * The protected method are for the MediaManager, since they affect the media
 * or the drivers.
 * TODO jsprevost: Change the inheritance to use atlas::ImageSequenceCapture
 */
class MediaStreamer : public atlas::Subject<const cv::Mat &>,
                      public atlas::Runnable {
 public:
  //==========================================================================
  // C L A S S   F R I E N D S H  I P

  /**
   * MediaManager needs more control over acquisition loop than normal users.
   */
  friend class MediaManager;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MediaStreamer>;

  //============================================================================
  // C O N S T A N T S   M E M B E R S

  static const char *LOOP_TAG;
  const int ARTIFICIAL_FRAMERATE = 15;  // FPS
  //==========================================================================
  // P U B L I C   C / D T O R S

  /**
   * Artificial frame rate simulate a frame rate for video and images.
   * It will run the loop at this speed.
   */
  explicit MediaStreamer(BaseMedia::Ptr cam, int artificialFrameRateMs = 30);

  virtual ~MediaStreamer();

  //==========================================================================
  // P U B L I C   M E T H O D S

  const std::string &GetMediaName() const;

  BaseMedia::Status GetMediaStatus() const;

  /**
   * Return either if the acquisition loop is streaming the video or not.
   *
   * \return True if the video feed is being streamed.
   */
  bool IsStreaming() const;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * Start/stop the loop.
   */
  bool StartStreaming();

  bool StopStreaming();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Acquisition thread, from HTThread
   * main loop to call NextImage on the media.
   */
  void Run() override;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * Protection of concurrency access between getImage and run.
   */
  mutable std::mutex image_access_;

  /**
   * Active media of the loop
   */
  BaseMedia::Ptr media_;

  /**
   * Keeping a reference to the most recent image.
   * This is not supposed to be an image owned by the media streamer
   * as it will call the swallow copy of the media image in order
   * to store it.
   */
  cv::Mat image_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const std::string &MediaStreamer::GetMediaName() const {
  return media_->GetName();
}

//------------------------------------------------------------------------------
//
inline bool MediaStreamer::IsStreaming() const {
  return media_->GetStatus() == BaseMedia::Status::STREAMING;
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_
