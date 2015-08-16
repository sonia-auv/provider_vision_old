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
Execution::Execution(
    atlas::NodeHandlePtr node_handle,
    std::shared_ptr<AcquisitionLoop> acquisition_loop,
    Filterchain *filterchain, const std::string &execName)
    : _acquisition_loop(acquisition_loop),
      _filterchain_to_process(filterchain),
      _state(CLOSE),
      image_publisher_(node_handle, VISION_NODE_NAME + execName + "_image"),
      result_publisher_(),
      _exec_name(VISION_NODE_NAME + execName),
      TRY_CLOSE(3),
      _new_image_ready(false) {
  assert(node_handle.get() != nullptr);
  image_publisher_.start();
  result_publisher_ =
      node_handle->advertise<std_msgs::String>(_exec_name + "_result", 50);

  if (_acquisition_loop.get() != nullptr)
    _camera_id = _acquisition_loop->GetMediaID();
}

//------------------------------------------------------------------------------
//
Execution::~Execution() {
  if (_state == RUNNING) StopExec();
  // No need to destroy _image_topic, it is a smart pointer
  ROS_INFO_NAMED(EXEC_TAG, "Destroying execution");
  unsigned int tries = 0;
  while (!_newest_image_mutex.try_lock()) {
    atlas::MilliTimer::sleep(20);
    tries++;
    if (tries > 100) {
      ROS_WARN_NAMED(EXEC_TAG,
                     "CANNOT UNLOCKED THE MUTEX BECAUSE IT IS LOCKED.");
      tries = 0;
    }
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
Execution::ERROR Execution::StartExec() {
  ERROR status = ERROR::FAILURE_TO_START;
  if (_acquisition_loop.get() == nullptr) {
    ROS_ERROR_NAMED(EXEC_TAG, "Acquisition loop is null!");
    return status;
  }

  if (this->_filterchain_to_process == nullptr) {
    ROS_ERROR_NAMED(EXEC_TAG, "Filterchain is null!");
    return status;
  }

  _acquisition_loop->Attach(*this);

  // Attach to the acquisition loop to receive notification from a new image.
  _filterchain_to_process->InitFilters();

  ROS_INFO_NAMED(EXEC_TAG, "Starting execution on %s with %s",
                 _filterchain_to_process->GetName().c_str(),
                 _camera_id.GetName().c_str());
  _state = RUNNING;
  status = ERROR::SUCCESS;

  // Start thread!
  start();

  return status;
}

//------------------------------------------------------------------------------
//
Execution::ERROR Execution::StopExec() {
  ERROR status = ERROR::FAILURE_TO_CLOSE;

  std::lock_guard<std::mutex> guard(_newest_image_mutex);
  _acquisition_loop->Detach(*this);

  // Stop thread
  if (thread_.joinable()) {
    stop();
  } else {
    ROS_WARN_NAMED(EXEC_TAG, "The excecution is not processing.");
  }

  _filterchain_to_process->CloseFilters();

  ROS_INFO_NAMED(EXEC_TAG, "Closing execution of %s with %s",
                 _filterchain_to_process->GetName().c_str(),
                 GetID().GetFullName());

  image_publisher_.stop();

  _state = CLOSE;
  status = ERROR::SUCCESS;

  return status;
}

//------------------------------------------------------------------------------
//
void Execution::OnSubjectNotify(atlas::Subject<> &subject) ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(_newest_image_mutex);
  _acquisition_loop->GetImage(_newest_image);
  _new_image_ready = true;
}

//------------------------------------------------------------------------------
//
void Execution::run() {
  while (!stop_) {
    // Prevent to process data twice for fast processing
    if (!_new_image_ready) {
      atlas::MilliTimer::sleep(10);
      continue;
    }
    _newest_image_mutex.lock();
    _image_being_processed = _newest_image.clone();
    _new_image_ready = false;
    _newest_image_mutex.unlock();

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

        image_publisher_.write(_image_being_processed);
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
