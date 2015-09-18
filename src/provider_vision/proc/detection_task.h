/**
 * \file	Execution.h
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_EXECUTION_H_
#define VISION_SERVER_EXECUTION_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <mutex>
#include <lib_atlas/ros/image_publisher.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/pattern/observer.h>
#include "provider_vision/config.h"
#include "provider_vision/proc/filterchain.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/media_streamer.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * DetectionTask class is responsible of taking the image from an acquisition
 * loop,
 * broadcast it on topic and apply the given filterchain.
 */
class DetectionTask : public atlas::Runnable, public atlas::Observer<> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  /**
   * Error handling
   */
  enum ERROR { SUCCESS = 0, FAILURE_TO_START, FAILURE_TO_CLOSE };

  /**
   * Internal state machine
   */
  enum STATE { RUNNING, CLOSE };

  //==========================================================================
  // P U B L I C   C / D T O R S

  /**
   * CTOR/DSTR
   */
  explicit DetectionTask(atlas::NodeHandlePtr node_handle,
                         std::shared_ptr<MediaStreamer> acquisition_loop,
                         Filterchain *filterchain, const std::string &execName);

  virtual ~DetectionTask();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Exectution command
   * Only start/stop the thread of image processing, not the acquisition loop.
   */
  ERROR StartExec();

  ERROR StopExec();

  /**
   * HTObserver override
   * Catches the acquisitionLoop's notification that an image is ready.
   */
  auto OnSubjectNotify(atlas::Subject<> &subject) ATLAS_NOEXCEPT
      -> void override;

  /**
     * HTThread override
     * Run the filterchain when the new image is ready.
     * Is necessary to decouple the acqusition loop from the processing.
   */
  void run() override;

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  const std::shared_ptr<MediaStreamer> GetAcquisitionLoop() const;

  const Filterchain *getFilterChain() const;

  const std::string GetExecName() const;

  const std::string GetMediaName() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * The publisher we use to send the filtered images on the ROS pipeline.
   *
   * We don't use the image transport provided publisher because we
   * extended the functionnalities of this one to be able to use the observer
   * pattern on the pubisher. The publisher is an observer and can be
   * attached to, for exemple, a subject that stream the content of a video
   * file.
   */
  atlas::ImagePublisher image_publisher_;

  /**
   * This publisher will send the result of the image processing of the
   * filterchain onto the ROS pipeline. It will contain informations about
   * a potentially found object.
   *
   * This is a simple string separated by comma in this way:
   * obstacle_name:x,y,width,height,specific_message;
   * This could also return several objects this way:
   * obstacle_name:x,y,width,height,specific_message;
   * x2,y2,width2,height2,specific_message2;
   */
  ros::Publisher result_publisher_;

  /**
   * DetectionTask core.
   */
  std::shared_ptr<MediaStreamer> media_streaming_;

  Filterchain *_filterchain_to_process;

  mutable std::mutex _newest_image_mutex;

  // Two image are needed since we don't want the mutex to block the process
  // on the image and we have to wait for it to be finish to release it so
  // the firenotification doesn't wait for a mutex.
  cv::Mat _newest_image, _image_being_processed;

  // Prevent to process data twice for fast processing
  bool _new_image_ready;
  /**
   * DetectionTask's media
   */
  STATE _state;

  const int TRY_CLOSE;

  std::string _exec_name;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const std::shared_ptr<MediaStreamer> DetectionTask::GetAcquisitionLoop()
    const {
  return media_streaming_;
}

//------------------------------------------------------------------------------
//
inline const Filterchain *DetectionTask::getFilterChain() const {
  return _filterchain_to_process;
}

//------------------------------------------------------------------------------
//
inline const std::string DetectionTask::GetExecName() const {
  return _exec_name;
}

//------------------------------------------------------------------------------
//
inline const std::string DetectionTask::GetMediaName() const {
    return media_streaming_.get()->GetMediaName();
}

}  // namespace vision_server

#endif  // VISION_SERVER_EXECUTION_H_
