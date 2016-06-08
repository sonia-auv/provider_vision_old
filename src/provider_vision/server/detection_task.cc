/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include "provider_vision/server/detection_task.h"
#include <ros/ros.h>
#include <sonia_msgs/VisionTarget.h>
#include <std_msgs/String.h>
#include "provider_vision/server/target.h"

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
      returning_orinal_image_(false) {
  assert(filterchain);
  assert(acquisition_loop);
  result_publisher_ = ros::NodeHandle().advertise<sonia_msgs::VisionTarget>(
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
  ROS_INFO("Starting detection task %s", this->name_.c_str());
}

//------------------------------------------------------------------------------
//
void DetectionTask::StopDetectionTask() {
  if (!IsRunning()) {
    throw std::logic_error("This detection task is not running.");
  }
  Stop();
  std::lock_guard<std::mutex> guard(newest_image_mutex_);
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
                                    const cv::Mat &image) {
  std::lock_guard<std::mutex> guard(newest_image_mutex_);
  newest_image_ = image;
  new_image_ready_ = true;
}

//------------------------------------------------------------------------------
//
void DetectionTask::Run() {
  while (!MustStop()) {
    try {
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
        filterchain_->ExecuteFilterChain(image_being_processed_);

        // We don't want to send stuff for nothing.
        if (!image_being_processed_.empty()) {
          if (image_being_processed_.depth() != CV_8U) {
            image_being_processed_.convertTo(image_being_processed_, CV_8U);
          }

          if (image_being_processed_.channels() == 1) {
            cv::cvtColor(image_being_processed_, image_being_processed_,
                         CV_GRAY2BGR);
          }
          try {
            image_publisher_.Write(image_being_processed_);
          } catch (std::exception &e) {
            // Sync issue... gotta go testing, will solve another time (famous
            // last word)
            // Throw me rocks...
            // signed Jeremie St-Jules
          }
        }
        provider_vision::GlobalParamHandler *paramHandler =
            filterchain_->GetParameterHandler();
        if (paramHandler) {
          provider_vision::TargetQueue targetQueue =
              paramHandler->getTargetQueue();
          while (!targetQueue.empty()) {
            sonia_msgs::VisionTarget msg;
            provider_vision::Target target = targetQueue.front();
            target.SetMessage(msg);
            result_publisher_.publish(msg);
            targetQueue.pop();
          }
          paramHandler->clearTarget();
        }
      } else {
        image_publisher_.Write(image_being_processed_);
      }
    } catch (std::exception &e) {
      ROS_ERROR("CATCHED ERROR IN DETEC TASK");
    }
  }
}

}  // namespace provider_vision
