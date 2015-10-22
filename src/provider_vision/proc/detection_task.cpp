/**
 * \file	Execution.cpp
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "provider_vision/proc/detection_task.h"

namespace vision_server {

//==============================================================================
// C O N S T A N T S   S E C T I O N

const std::string DetectionTask::EXEC_TAG = "[EXECUTION]";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DetectionTask::DetectionTask(MediaStreamer::Ptr acquisition_loop,
                             Filterchain::Ptr filterchain,
                             const std::string &execution_name)
    : name_(kRosNodeName + execution_name),
      image_publisher_(kRosNodeName + execution_name + "_image"),
      result_publisher_(),
      media_streamer_(acquisition_loop),
      filterchain_(filterchain),
      new_image_ready_(false),
      close_attemps_(3) {
  assert(filterchain);
  assert(acquisition_loop);
  result_publisher_ =
      ros::NodeHandle().advertise<std_msgs::String>(name_ + "_result", 50);
}

//------------------------------------------------------------------------------
//
DetectionTask::~DetectionTask() {
  if (IsRunning()) {
    StopDetectionTask();
  }
  ROS_INFO_NAMED(EXEC_TAG.c_str(), "Destroying execution");
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DetectionTask::StartDetectionTask() {
  if (IsRunning()) {
    throw std::logic_error("This detection task is already running.");
  }
  // Attach to the acquisition loop to receive notification from a new image.
  filterchain_->InitFilters();
  Start();
  image_publisher_.Start();
  media_streamer_->Attach(*this);
  ROS_INFO_NAMED(EXEC_TAG, "Starting detection task ");
}

//------------------------------------------------------------------------------
//
void DetectionTask::StopDetectionTask() {
  std::lock_guard<std::mutex> guard(newest_image_mutex_);
  if (!IsRunning()) {
    throw std::logic_error("This detection task is not running.");
  }
  media_streamer_->Detach(*this);
  image_publisher_.Stop();
  Stop();
  filterchain_->CloseFilters();
}

//------------------------------------------------------------------------------
//
void DetectionTask::OnSubjectNotify(atlas::Subject<const cv::Mat &> &subject,
                                    const cv::Mat &image) noexcept {
  std::lock_guard<std::mutex> guard(newest_image_mutex_);
  newest_image_ = image;
  new_image_ready_ = true;
}

//------------------------------------------------------------------------------
//
void DetectionTask::Run() {
  while (!MustStop()) {
    // Prevent to process data twice for fast processing
    if (!new_image_ready_) {
      atlas::MilliTimer::Sleep(1);
      continue;
    }
    newest_image_mutex_.lock();
    _image_being_processed = newest_image_.clone();
    new_image_ready_ = false;
    newest_image_mutex_.unlock();

    std::string return_string;

    return_string = filterchain_->ExecuteFilterChain(_image_being_processed);

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

}  // namespace vision_server
