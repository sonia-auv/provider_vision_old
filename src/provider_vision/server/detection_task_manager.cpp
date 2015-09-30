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
std::vector<std::shared_ptr<DetectionTask>>
DetectionTaskManager::GetAllDetectionTasks() const noexcept {
  return detection_tasks_;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<DetectionTask> DetectionTaskManager::GetDetectionTask(
    const std::string &execution_name) const {
  for (const auto &task : detection_tasks_) {
    if (task->GetName() == execution_name) {
      return task;
    }
  }
  throw std::invalid_argument("This detection task does not exists.");
}

//------------------------------------------------------------------------------
//
std::shared_ptr<DetectionTask> DetectionTaskManager::CreateDetectionTask(
    std::shared_ptr<ros::NodeHandle> node_handle,
    std::shared_ptr<MediaStreamer> acquisition_loop,
    std::shared_ptr<Filterchain> filterchain,
    const std::string &execution_name) noexcept {
  auto task = std::make_shared<DetectionTask>(node_handle, acquisition_loop,
                                              filterchain, execution_name);
  detection_tasks_.push_back(task);
  return task;
}

//------------------------------------------------------------------------------
//
void DetectionTaskManager::StopDetectionTask(
    const std::string &execution_name) noexcept {
  const auto &it = detection_tasks_.cbegin();
  for (; it != detection_tasks_.cend(); std::advance(it, 1)) {
    if ((*it)->GetName() == execution_name) {
      break;
    }
  }
  if (it != detection_tasks_.cend()) {
    (*it)->stop();
    detection_tasks_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
void DetectionTaskManager::StopDetectionTask(
    std::shared_ptr<DetectionTask> task) noexcept {
  const auto &it = detection_tasks_.cbegin();
  for (; it != detection_tasks_.cend(); std::advance(it, 1)) {
    if ((*it).get() == task.get()) {
      break;
    }
  }
  if (it != detection_tasks_.cend()) {
    (*it)->stop();
    detection_tasks_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
std::vector<std::shared_ptr<MediaStreamer>>
DetectionTaskManager::GetAllMediaStreamers() const noexcept {
  std::vector<std::shared_ptr<MediaStreamer>> media_streamers = {};
  for (const auto &task : detection_tasks_) {
    media_streamers.push_back(task->GetMediaStreamer());
  }
  return media_streamers;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<MediaStreamer> DetectionTaskManager::GetMediaStreamer(
    const std::string &execution_name, const std::string &media_name) const
    noexcept {
  return GetDetectionTask(execution_name)->GetMediaStreamer();
}

//------------------------------------------------------------------------------
//
std::shared_ptr<Filterchain> DetectionTaskManager::GetRunningFilterchains(
    const std::string &execution_name) const noexcept {
  return GetDetectionTask(execution_name)->GetFilterchain();
}

//------------------------------------------------------------------------------
//
std::vector<std::shared_ptr<Filterchain>>
DetectionTaskManager::GetAllRunningFilterchains() const noexcept {
  std::vector<std::shared_ptr<Filterchain>> v = {};
  for(const auto &task : detection_tasks_) {
    v.push_back(task->GetFilterchain());
  }
  return v;
}
//------------------------------------------------------------------------------
//
bool DetectionTaskManager::IsMediaUsed(const std::string &media_name) {
  for(const auto &task : detection_tasks_) {
    if(task->GetMediaName() == media_name) {
      return true;
    }
  }
  return false;
}

}  // namespace vision_server
