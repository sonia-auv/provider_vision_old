/**
 * \file	Execution.cpp
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <std_msgs/String.h>
#include <lib_atlas/macros.h>
#include "provider_vision/proc/detection_task.h"

namespace vision_server {

//==============================================================================
// C O N S T A N T S   S E C T I O N

const std::string DetectionTask::EXEC_TAG = "[EXECUTION]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DetectionTask::DetectionTask(std::shared_ptr<ros::NodeHandle> node_handle,
                             MediaStreamer::Ptr acquisition_loop,
                             Filterchain::Ptr filterchain,
                             const std::string &execution_name)
    : name_(kRosNodeName + execution_name),
      image_publisher_(node_handle, kRosNodeName + execution_name + "_image"),
      result_publisher_(),
      media_streamer_(acquisition_loop),
      filterchain_(filterchain),
      _new_image_ready(false),
      running_(false),
      close_attemps_(3) {
  assert(node_handle.get() != nullptr);
  image_publisher_.Start();
  result_publisher_ =
      node_handle->advertise<std_msgs::String>(name_ + "_result", 50);

  if (media_streamer_.get() == nullptr)
    throw std::invalid_argument("media_streaming is null");
}

//------------------------------------------------------------------------------
//
DetectionTask::~DetectionTask() {
  if (running_ == true) Stop();
  // No need to destroy _image_topic, it is a smart pointer
  ROS_INFO_NAMED(EXEC_TAG.c_str(), "Destroying execution");
  unsigned int tries = 0;
  while (!_newest_image_mutex.try_lock()) {
    atlas::MilliTimer::Sleep(20);
    tries++;
    if (tries > 100) {
      ROS_WARN_NAMED(EXEC_TAG.c_str(),
                     "CANNOT UNLOCKED THE MUTEX BECAUSE IT IS LOCKED.");
      tries = 0;
    }
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DetectionTask::Start() {
  if (media_streamer_.get() == nullptr) {
    throw std::runtime_error("Cannot start task with null media streamer");
  }

  if (filterchain_ == nullptr) {
    throw std::runtime_error("Cannot start task with null filterchain");
  }

  if (IsRunning()) {
    throw std::logic_error("This excecution is already running.");
  }

  media_streamer_->Attach(*this);

  // Attach to the acquisition loop to receive notification from a new image.
  filterchain_->InitFilters();

  ROS_INFO_NAMED(EXEC_TAG, "Starting execution ");
  running_ = true;
  Runnable::Start();
}

//------------------------------------------------------------------------------
//
void DetectionTask::Stop() {
  std::lock_guard<std::mutex> guard(_newest_image_mutex);
  if (!IsRunning()) {
    throw std::logic_error("This excecution is not running.");
  }

  Runnable::Stop();
  media_streamer_->Detach(*this);
  filterchain_->CloseFilters();
  image_publisher_.Stop();
  running_ = false;
}

//------------------------------------------------------------------------------
//
void DetectionTask::OnSubjectNotify(atlas::Subject<> &subject) noexcept {
  std::lock_guard<std::mutex> guard(_newest_image_mutex);
  media_streamer_->GetImage(_newest_image);
  _new_image_ready = true;
}

//------------------------------------------------------------------------------
//
void DetectionTask::Run() {
  while (!MustStop()) {
    // Prevent to process data twice for fast processing
    if (!_new_image_ready) {
      atlas::MilliTimer::Sleep(10);
      continue;
    }
    _newest_image_mutex.lock();
    _image_being_processed = _newest_image.clone();
    _new_image_ready = false;
    _newest_image_mutex.unlock();

    std::string return_string;

    if (running_ == true) {
      if (filterchain_ != nullptr) {
        return_string =
            filterchain_->ExecuteFilterChain(_image_being_processed);
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

        image_publisher_.Write(_image_being_processed);
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
