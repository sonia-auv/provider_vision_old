/**
 * \file	media_streamer.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_
#define PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_

#include <mutex>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <lib_atlas/sys/timer.h>
#include <lib_atlas/sys/fsinfo.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/pattern/runnable.h>
#include "provider_vision/media/camera/base_media.h"

namespace vision_server {

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

  static const std::string LOOP_TAG;

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

  /**
   * Get the most recent image.
   * TODO jsprevost: Change the return type to cv::Mat
   */
  void GetImage(cv::Mat &image) const;

  const std::string &GetMediaName() const;

  BaseMedia::Status GetMediaStatus() const;

  /**
   * Return either if the acquisition loop is recording the video or not.
   *
   * \return True if the video feed is being record.
   */
  bool IsRecording() const;

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
   * Those method can only be use by MediaManager, as it affect the Media
   * and drivers also !
   * Set the framerate of the thread. Comes with the a change of framerate
   * for the media.
   */
  void SetFramerate(int framePerSecond);

  /**
   * Start/stop the loop.
   */
  void StartStreaming();

  void StopStreaming();

  /**
   * If the camera is a Real camera, we want to record a video feed for test
   * and debugging purpose.
   * After having test if the camera is a real camera on the runtion,
   * the acquisition loop will start the record of the camera.
   * The recording will take end at the destruction of the acquisition loop or
   *at
   * the end of the ThredFunction.
   *
   * \return True if the record can be processed, False if something went wrong.
   */
  void StartRecording(const std::string &filename = "");

  /**
   * Stop the record of the camera by closing the file the image are saved to
   * and setting the acquisition flag for record to false.
   *
   * \return True if the function closed the file.
   */
  void StopRecording();

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
   * FrameRate members
   */
  int artificial_framerate_;

  int framerate_mili_sec_;

  /**
   * Keeping a reference to the most recent image.
   * This is not supposed to be an image owned by the media streamer
   * as it will call the swallow copy of the media image in order
   * to store it.
   */
  cv::Mat image_;

  /**
   * This is the actuall VideoWriter that allows us to record the video
   * in a specific file.
   * This also provide a isOpened method that must be used before performing
   * writing operations in it.
   */
  cv::VideoWriter video_writer_;

  /**
   * If the media is a real camera (not a video neither a webcam), we want to
   * save the feed for test purpose.
   * The default behavior is to set this flag to true when starting the stream.
   * If you don't want to save the received images (over processing), simply
   * change this behavior into the StartStreaming function.
   */
  bool is_recording_;

  mutable std::mutex list_access_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const std::string &MediaStreamer::GetMediaName() const {
  return media_->GetName();
};

//------------------------------------------------------------------------------
//
inline bool MediaStreamer::IsRecording() const {
  return is_recording_ && video_writer_.isOpened();
}

//------------------------------------------------------------------------------
//
inline bool MediaStreamer::IsStreaming() const {
  return media_->GetStatus() == BaseMedia::Status::STREAMING;
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_ACQUISITION_LOOP_H_
