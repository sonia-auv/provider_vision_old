/**
 * \file	detection_task.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_PROC_DETECTION_TASK_H_
#define PROVIDER_VISION_PROC_DETECTION_TASK_H_

#include <lib_atlas/pattern/observer.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/ros/image_publisher.h>
#include <mutex>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/media_streamer.h"
#include "provider_vision/server/filterchain.h"
#include "provider_vision/config.h"

namespace provider_vision {

/**
 * DetectionTask class is responsible of taking the image from an acquisition
 * loop,
 * broadcast it on topic and apply the given filterchain.
 */
class DetectionTask : private atlas::Runnable,
                      public atlas::Observer<const cv::Mat &> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DetectionTask>;

  static const std::string EXEC_TAG;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit DetectionTask(MediaStreamer::Ptr acquisition_loop,
                         Filterchain::Ptr filterchain,
                         const std::string &execution_name);

  virtual ~DetectionTask();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void StartDetectionTask();

  void StopDetectionTask();

  /**
   * Change the image returned by the detection task to the result of a
   * specific filter.
   * This behavior is being handled by the filterchain so this method is
   * a wrapper around the filterchain method that set the observed filter.
   */
  void ChangeReturnImageToFilter(const size_t &index);

  /**
   * Change the image returned by the detection task to the filterchain returned
   * image.
   * This is the default behavior, the image returned is the result of the
   * whole pipeline of filters.
   */
  void ChangeReturnImageToFilterchain();

  /**
   * Change the image returned by the detection task to the original image.
   * If this parameters is enables, the image is not being processed and
   * the image that is being sent on the network is the original image
   * from the media.
   */
  void ChangeReturnImageToOrigin();

  MediaStreamer::Ptr GetMediaStreamer() const noexcept;

  Filterchain::Ptr GetFilterchain() const noexcept;

  const std::string &GetDetectionTaskName() const noexcept;

  const std::string &GetMediaName() const noexcept;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * HTObserver override
   * Catches the acquisitionLoop's notification that an image is ready.
   */
  void OnSubjectNotify(atlas::Subject<const cv::Mat &> &subject,
                       const cv::Mat &image) noexcept override;

  /**
   * HTThread override
   * Run the filterchain when the new image is ready.
   * Is necessary to decouple the acqusition loop from the processing.
   */
  void Run() override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string name_;

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

  MediaStreamer::Ptr media_streamer_;

  Filterchain::Ptr filterchain_;

  mutable std::mutex newest_image_mutex_;

  /**
   * Two image are needed since we don't want the mutex to block the process
   * on the image and we have to wait for it to be finish to release it so
   * the firenotification doesn't wait for a mutex.
   */
  cv::Mat newest_image_;

  cv::Mat image_being_processed_;

  /** Prevent to process data twice for fast processing */
  bool new_image_ready_;

  bool returning_orinal_image_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline MediaStreamer::Ptr DetectionTask::GetMediaStreamer() const noexcept {
  return media_streamer_;
}

//------------------------------------------------------------------------------
//
inline Filterchain::Ptr DetectionTask::GetFilterchain() const noexcept {
  return filterchain_;
}

//------------------------------------------------------------------------------
//
inline const std::string &DetectionTask::GetDetectionTaskName() const noexcept {
  return name_;
}

//------------------------------------------------------------------------------
//
inline const std::string &DetectionTask::GetMediaName() const noexcept {
  return media_streamer_.get()->GetMediaName();
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_PROC_DETECTION_TASK_H_
