/**
 * \file	detection_task.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "provider_vision/proc/detection_task.h"

namespace provider_vision {

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
    : name_(execution_name),
      image_publisher_(kRosNodeName + execution_name + "_image"),
      result_publisher_(),
      media_streamer_(acquisition_loop),
      filterchain_(filterchain),
      new_image_ready_(false),
      returning_orinal_image_(false),
      close_attemps_(3) {
  assert(filterchain);
  assert(acquisition_loop);
  result_publisher_ = ros::NodeHandle().advertise<std_msgs::String>(
      kRosNodeName + name_ + "_result", 50);
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
  Stop();
  result_publisher_.shutdown();
  media_streamer_->Detach(*this);
  image_publisher_.Stop();
}

//------------------------------------------------------------------------------
//
void DetectionTask::ChangeReturnImageToFilter(const size_t &index) {
  returning_orinal_image_ = false;
  filterchain_->SetObserver(index);
}

//------------------------------------------------------------------------------
//
void DetectionTask::ChangeReturnImageToFilterchain() {
  returning_orinal_image_ = false;
  auto last_index = filterchain_->GetAllFilters().size() - 1;
  ChangeReturnImageToFilter(last_index);
}

//------------------------------------------------------------------------------
//
void DetectionTask::ChangeReturnImageToOrigin() {
  returning_orinal_image_ = true;
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
    try
    {

      // Prevent to process data twice for fast processing
      if (!new_image_ready_) {
        atlas::MilliTimer::Sleep(1);
        continue;
      }
      newest_image_mutex_.lock();
      image_being_processed_ = newest_image_.clone();
      new_image_ready_ = false;
      newest_image_mutex_.unlock();

      if (!returning_orinal_image_) {
        std::string return_string;
          return_string = filterchain_->ExecuteFilterChain(image_being_processed_);

        // We don't want to send stuff for nothing.
        if (!image_being_processed_.empty()) {
          if (image_being_processed_.depth() != CV_8U) {
            image_being_processed_.convertTo(image_being_processed_, CV_8U);
          }

          if (image_being_processed_.channels() == 1) {
            cv::cvtColor(image_being_processed_, image_being_processed_,
                         CV_GRAY2BGR);
          }
          try{
            image_publisher_.Write(image_being_processed_);
          }catch(std::exception &e)
          {
           // Sync issue... gotta go testing, will solve another time (famous last word)
            // Throw me rocks...
            // signed Jeremie St-Jules
          }
        }
        if (return_string != "") {
          std_msgs::String msg;
          msg.data = return_string.c_str();
          result_publisher_.publish(msg);
        }
      } else {
          image_publisher_.Write(image_being_processed_);
      }
    }catch(std::exception &e){ ROS_ERROR("CATCHED ERROR IN DETEC TASK");}
  }
}

}  // namespace provider_vision
