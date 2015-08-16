/**
 * \file	Execution.cpp
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <assert.h>
#include <std_msgs/String.h>
#include <CLTimer.h>
#include <lib_atlas/typedef.h>
#include "server/execution.h"

namespace vision_server {

//==============================================================================
// C O N S T A N T S   S E C T I O N

static const char *EXEC_TAG = "[EXECUTION]";

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_server::Execution::Execution(
    atlas::NodeHandlePtr node_handle,
    vision_server::AcquisitionLoop::Ptr acquisition_loop,
    Filterchain *filterchain, const std::string &execName)
    : _acquisition_loop(acquisition_loop),
      _filterchain_to_process(filterchain),
      _state(CLOSE),
      _image_topic(nullptr),
      result_publisher_(),
      _exec_name(VISION_NODE_NAME + execName),
      TRY_CLOSE(3),
      _new_image_ready(false) {
  assert(node_handle.get() != nullptr);

  _image_topic = new ROSImageTopic(node_handle, _exec_name + "_image");
  result_publisher_ =
      node_handle->advertise<std_msgs::String>(_exec_name + "_result", 50);

  _newest_image_mutex.Create();

  if (_acquisition_loop.IsNotNull())
    _camera_id = _acquisition_loop->GetMediaID();

  // init filterchain
  //_filterchain_to_process
}

//------------------------------------------------------------------------------
//
vision_server::Execution::~Execution() {
  if (_state == RUNNING) StopExec();
  // No need to destroy _image_topic, it is a smart pointer
  ROS_INFO_NAMED(EXEC_TAG, "Destroying execution");
  unsigned int tries = 0;
  while (_newest_image_mutex.IsLocked()) {
    CLTimer::Delay(20);
    tries++;
    if (tries > 100) {
      ROS_WARN_NAMED(EXEC_TAG,
                     "CANNOT UNLOCKED THE MUTEX BECAUSE IT IS LOCKED.");
      tries = 0;
    }
  }
  _newest_image_mutex.Destroy();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
vision_server::Execution::ERROR vision_server::Execution::StartExec() {
  ERROR status = ERROR::FAILURE_TO_START;
  if (_acquisition_loop.IsNull()) {
    ROS_ERROR_NAMED(EXEC_TAG, "Acquisition loop is null!");
    return status;
  }

  if (this->_filterchain_to_process == nullptr) {
    ROS_ERROR_NAMED(EXEC_TAG, "Filterchain is null!");
    return status;
  }

  if (_acquisition_loop->RegisterExecution(*this)) {
    ROS_INFO_NAMED(EXEC_TAG,
                   "%s is attached to the acquisition loop that run on %s",
                   _filterchain_to_process->GetName().c_str(),
                   _camera_id.GetName().c_str());
  } else {
    ROS_INFO_NAMED(EXEC_TAG,
                   "%s is NOT attached to the acquisition loop that run on %s",
                   _filterchain_to_process->GetName().c_str(),
                   _camera_id.GetName().c_str());
    return status;
  }

  // Create the topic on which we will put the image from the filter.
  _image_topic->OpenTopic();

  // Attach to the acquisition loop to receive notification from a new image.
  _filterchain_to_process->InitFilters();

  ROS_INFO_NAMED(EXEC_TAG, "Starting execution on %s with %s",
                 _filterchain_to_process->GetName().c_str(),
                 _camera_id.GetName().c_str());
  _state = RUNNING;
  status = ERROR::SUCCESS;

  // Start thread!
  Start();

  return status;
}

//------------------------------------------------------------------------------
//
vision_server::Execution::ERROR vision_server::Execution::StopExec() {
  ERROR status = ERROR::FAILURE_TO_CLOSE;

  CLMutex::Guard guard(_newest_image_mutex);
  // Do not act on it for now... simply log it...
  if (!_acquisition_loop->UnRegisterExecution(*this)) {
    ROS_ERROR_NAMED(EXEC_TAG, "Cannot detach from acquistion loop with %s",
                    GetID().GetFullName());
  }

  // Stop thread
  if (IsAlive()) {
    if (!Terminate()) {
      Kill();
    }
  } else {
    ROS_WARN_NAMED(EXEC_TAG, "Thread is not alive");
  }

  _filterchain_to_process->CloseFilters();

  ROS_INFO_NAMED(EXEC_TAG, "Closing execution of %s with %s",
                 _filterchain_to_process->GetName().c_str(),
                 GetID().GetFullName());

  _image_topic->CloseTopic();

  _state = CLOSE;
  status = ERROR::SUCCESS;

  return status;
}

//------------------------------------------------------------------------------
//
void vision_server::Execution::OnSubjectNotify(HTSubject *hook) {
  CLMutex::Guard guard(_newest_image_mutex);
  _acquisition_loop->GetImage(_newest_image);
  _new_image_ready = true;
}

//------------------------------------------------------------------------------
//
void vision_server::Execution::ThreadFunc() {
  while (!ThreadMustExit()) {
    // Prevent to process data twice for fast processing
    if (!_new_image_ready) {
      CLTimer::Delay(10);
      continue;
    }
    _newest_image_mutex.Take();
    _image_being_processed = _newest_image.clone();
    _new_image_ready = false;
    _newest_image_mutex.Release();

    std::string return_string;

    if (_state == RUNNING) {
      if (_filterchain_to_process != nullptr) {
        return_string =
            _filterchain_to_process->ExecuteFilterChain(_image_being_processed);
      }

      // We don't want to send stuff for nothing.
      if (!_image_being_processed.empty()) {
        if (_image_being_processed.depth() != CV_8U) {
          _image_being_processed.convertTo(_image_being_processed, CV_8U);
        }

        if (_image_being_processed.channels() == 1) {
          cv::cvtColor(_image_being_processed, _image_being_processed,
                       CV_GRAY2BGR);
        }

        _image_topic->PublishImage(_image_being_processed);
      }
      if (return_string != "") {
        std_msgs::String msg;
        msg.data = return_string.c_str();
        result_publisher_.publish(msg);
      }
    }
  }
}

}  // namespace vision_server
