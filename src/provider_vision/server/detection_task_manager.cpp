/**
 * \file	detection_task_manager.cpp
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/server/detection_task_manager.h"

namespace vision_server {

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
    std::shared_ptr<ros::NodeHandle> node_handle,
    MediaStreamer::Ptr media_streamer, Filterchain::Ptr filterchain,
    const std::string &execution_name) noexcept {
  try {
    GetDetectionTask(execution_name);
  } catch (const std::exception &e) {
    std::cout << "Execution already exist" << std::endl;
  }

  auto task = std::make_shared<DetectionTask>(node_handle, media_streamer,
                                              filterchain, execution_name);
  detection_tasks_.push_back(task);
}

//------------------------------------------------------------------------------
//
void DetectionTaskManager::StopDetectionTask(
    const std::string &execution_name) noexcept {
  for (auto it = detection_tasks_.begin(); it != detection_tasks_.cend();
       it++) {
    if ((*it)->GetDetectionTaskName().compare(execution_name) == 0) {
        (*it)->StopDetectionTask();
      detection_tasks_.erase(it);
    }
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
DetectionTask::Ptr DetectionTaskManager::GetDetectionTask(
    const std::string &execution_name) const {
  for (const auto &task : detection_tasks_) {
    if (task->GetDetectionTaskName() == execution_name) {
      return task;
    }
  }
  throw std::invalid_argument("This detection task does not exists.");
}

}  // namespace vision_server
