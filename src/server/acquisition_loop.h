/**
 * \file	AcquisitionLoop.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	12/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_ACQUISITION_LOOP_H_
#define VISION_SERVER_ACQUISITION_LOOP_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <HTThread.h>
#include <HTSmartPtr.h>
#include <HTSubject.h>
#include <CLMutex.h>

#include <opencv2/core/core.hpp>
#include "media/media.h"
#include "utils/camera_id.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Class responsible of acquiring an image from a device, asynchronously from
 * the system. It is basically a thread running and getting images from a media.
 * The protected method are for the CameraManager, since they affect the media
 * or the drivers.
 */
class AcquisitionLoop : public HTSubject, public HTSmartObj, private HTThread {
 public:
  //============================================================================
  // C O N S T A N T S   M E M B E R S

  const char *LOOP_TAG;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  /**
   * Smart pointer to an a acquisition loop object
   */
  typedef HTSmartPtr<AcquisitionLoop> Ptr;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Artificial frame rate simulate a frame rate for video and images.
   * It will run the loop at this speed.
   */
  AcquisitionLoop(Media::Ptr cam, int artificialFrameRateMs = 30);

  virtual ~AcquisitionLoop();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Get the most recent image.
   */
  bool GetImage(cv::Mat &image);

  /**
   * Get the media CameraID on which we take the images.
   */
  const CameraID GetMediaID();

  /**
   * Return the acquisition loop status.
   */
  const STATUS GetMediaStatus();

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

  /**
     * Add execution to the list
     */
  bool RegisterExecution(HTObsBase &exec);
  /**
     * Remove execution from the list
     */
  bool UnRegisterExecution(HTObsBase &exec);

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
     * Fire the notification
     */
  void InformExecution();

  /**
   * Those method can only be use by CameraManager, as it affect the Media
   * and drivers also !
   * Set the framerate of the thread. Comes with the a change of framerate
   * for the media.
   */
  void SetFramerate(int framePerSecond);

  /**
   * Start/stop the loop.
   */
  bool StartStreaming();

  bool StopStreaming();

  /**
   * If the camera is a Real camera, we want to record a video feed for test
   * and debugging purpose.
   * After having test if the camera is a real camera on the ThreadFunction,
   * the acquisition loop will start the record of the camera.
   * The recording will take end at the destruction of the acquisition loop or
   *at
   * the end of the ThredFunction.
   *
   * \return True if the record can be processed, False if something went wrong.
   */
  bool StartRecording(const std::string &filename = "");

  /**
   * Stop the record of the camera by closing the file the image are saved to
   * and setting the acquisition flag for record to false.
   *
   * \return True if the function closed the file.
   */
  bool StopRecording();

  /**
   * Returns the media name... to be deleted since we have the GetMediaID()
   * method.
   */
  const std::string GetMediaName();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Acquisition thread, from HTThread
   * main loop to call NextImage on the media.
   */
  void ThreadFunc() override;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * Flag to know if the loop is streaming.
   */
  bool _is_streaming;

  /**
   * Protection of concurrency access between getImage and ThreadFunc.
   */
  CLMutex _image_access;

  /**
   * Active media of the loop
   */
  Media::Ptr _media;

  /**
   * Logger
   */

  /**
   * FrameRate members
   */
  int _artificialFrameRate;

  int _frameRateMiliSec;

  /**
   * Most updated image.
   */
  cv::Mat _image;

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

  CLMutex list_access_;

  //==========================================================================
  // C L A S S   F R I E N D S H  I P

  /**
   * CameraManager needs more control over acquisition loop than normal users.
   */
  friend class CameraManager;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const std::string AcquisitionLoop::GetMediaName() {
  return _media->GetCameraID().GetName();
};

//------------------------------------------------------------------------------
//
inline bool AcquisitionLoop::IsRecording() const {
  return is_recording_ && video_writer_.isOpened();
}

//------------------------------------------------------------------------------
//
inline bool AcquisitionLoop::IsStreaming() const { return _is_streaming; }

//------------------------------------------------------------------------------
//
inline bool AcquisitionLoop::RegisterExecution(HTObsBase &exec) {
  CLMutex::Guard g(list_access_);
  return this->AttachObserver(exec);
}

//------------------------------------------------------------------------------
//
inline bool AcquisitionLoop::UnRegisterExecution(HTObsBase &exec) {
  CLMutex::Guard g(list_access_);
  this->DetachObserver(exec);
  return true;
}

//------------------------------------------------------------------------------
//
inline void AcquisitionLoop::InformExecution() {
  CLMutex::Guard g(list_access_);
  FireNotifications();
}

}  // namespace vision_server

#endif  // VISION_SERVER_ACQUISITION_LOOP_H_
