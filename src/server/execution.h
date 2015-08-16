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

#include <HTSmartPtr.h>
#include <HTObsBase.h>
#include <HTThread.h>
#include <CLMutex.h>
#include "media/media.h"
#include "config.h"

#include "ros/ros_image_topic.h"
#include "server/filterchain.h"
#include "server/acquisition_loop.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Execution class is responsible of taking the image from an acquisition loop,
 * broadcast it on topic and apply the given filterchain.
 */
class Execution : public HTObsBase, public HTSmartObj, public HTThread {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  /**
   * Defining a type for ease of use
   */
  typedef HTSmartPtr<Execution> Ptr;

  /**
   * Error handling
   */
  enum ERROR { SUCCESS = 0, FAILURE_TO_START, FAILURE_TO_CLOSE };

  /**
   * Internal state machine
   */
  enum STATE { RUNNING, CLOSE };

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * CTOR/DSTR
   */
  Execution(atlas::NodeHandlePtr node_handle, AcquisitionLoop::Ptr acquisition_loop, Filterchain *filterchain,
            const std::string &execName);

  virtual ~Execution();

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
  void OnSubjectNotify(HTSubject *hook) override;

  /**
     * HTThread override
     * Run the filterchain when the new image is ready.
     * Is necessary to decouple the acqusition loop from the processing.
   */
  void ThreadFunc() override;

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  const AcquisitionLoop::Ptr GetAcquisitionLoop() const;

  const Filterchain *getFilterChain() const;

  const CameraID GetID() const;

  const std::string GetExecName() const;

  const std::string GetMediaName() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ROSImageTopic::Ptr _image_topic;

  /**
   * This publisher will send the result of the image processing of the
   * filterchain onto the ROS pipeline. It will contain informations about
   * a potentially found object.
   * This is a simple string separated by comma in this way:
   * obstacle_name:x,y,width,height,specific_message;
   * This could also return several objects this way:
   * obstacle_name:x,y,width,height,specific_message;
   * x2,y2,width2,height2,specific_message2;
   */
  ros::Publisher result_publisher_;

  /**
   * Execution core.
   */
  AcquisitionLoop::Ptr _acquisition_loop;

  Filterchain *_filterchain_to_process;

  CLMutex _newest_image_mutex;
  // Two image are needed since we don't want the mutex to block the process
  // on the image and we have to wait for it to be finish to release it so
  // the firenotification doesn't wait for a mutex.
  cv::Mat _newest_image, _image_being_processed;
  // Prevent to process data twice for fast processing
  bool _new_image_ready;
  /**
   * Execution's media
   */
  CameraID _camera_id;

  STATE _state;

  const int TRY_CLOSE;

  std::string _exec_name;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const AcquisitionLoop::Ptr Execution::GetAcquisitionLoop() const {
  return _acquisition_loop;
}

//------------------------------------------------------------------------------
//
inline const Filterchain *Execution::getFilterChain() const {
  return _filterchain_to_process;
}

//------------------------------------------------------------------------------
//
inline const CameraID Execution::GetID() const { return _camera_id; }

//------------------------------------------------------------------------------
//
inline const std::string Execution::GetExecName() const { return _exec_name; }

//------------------------------------------------------------------------------
//
inline const std::string Execution::GetMediaName() const {
  return _camera_id.GetName();
}

}  // namespace vision_server

#endif  // VISION_SERVER_EXECUTION_H_
