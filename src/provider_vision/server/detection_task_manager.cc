/**
 * \file	detection_task_manager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/server/detection_task_manager.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
DetectionTaskManager::DetectionTaskManager() {}

//------------------------------------------------------------------------------
//
DetectionTaskManager::~DetectionTaskManager() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void DetectionTaskManager::StartDetectionTask(
    MediaStreamer::Ptr media_streamer, Filterchain::Ptr filterchain,
    const std::string &execution_name) {
  if (execution_name.empty()) {
    throw std::invalid_argument("The detection task name is not valid");
  }
  DetectionTask::Ptr task = GetDetectionTask(execution_name);
  if (task == nullptr) {
    task = std::make_shared<DetectionTask>(media_streamer, filterchain,
                                           execution_name);
    task->StartDetectionTask();
    detection_tasks_.push_back(task);
  } else {
    throw std::logic_error("This detection task already exist.");
  }
}

//------------------------------------------------------------------------------
//
void DetectionTaskManager::StopDetectionTask(
    const std::string &execution_name) {
  auto detection_task = GetDetectionTask(execution_name);
  auto it = std::find(detection_tasks_.begin(), detection_tasks_.end(), detection_task);
  if(it != detection_tasks_.end()) {
    detection_tasks_.erase(it);
  } else {
    throw std::invalid_argument("This detection taks does not exist");
  }
}

//------------------------------------------------------------------------------
//
std::vector<std::string> DetectionTaskManager::GetAllDetectionTasksName() const
    noexcept {
  std::vector<std::string> names;
  for (const auto &task : detection_tasks_) {
    names.push_back(task->GetDetectionTaskName());
  }
  return names;
}

//------------------------------------------------------------------------------
//
size_t DetectionTaskManager::GetAllDetectionTasksCount() const noexcept {
  return detection_tasks_.size();
}

//------------------------------------------------------------------------------
//
MediaStreamer::Ptr DetectionTaskManager::GetMediaStreamerFromDetectionTask(
    const std::string &name) const noexcept {
  return GetDetectionTask(name)->GetMediaStreamer();
}

//------------------------------------------------------------------------------
//
Filterchain::Ptr DetectionTaskManager::GetFilterchainFromDetectionTask(
    const std::string &name) const noexcept {
  return GetDetectionTask(name)->GetFilterchain();
}

//------------------------------------------------------------------------------
//
DetectionTask::Ptr DetectionTaskManager::GetDetectionTask(
    const std::string &execution_name) const {
  for (const auto &task : detection_tasks_) {
    if (task->GetDetectionTaskName() == execution_name) {
      return task;
    }
  }
  return nullptr;
}

}  // namespace provider_vision
