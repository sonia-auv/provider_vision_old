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

//------------------------------------------------------------------------------
//
std::shared_ptr<DetectionTask> DetectionTaskManager::GetDetectionTask(
    const std::string &execName) {
  std::lock_guard<std::mutex> guard(_list_access);
  for (const auto &execution : detection_tasks_) {
    if (execution->GetExecName().find(execName.c_str()) != std::string::npos) {
      return execution;
    }
  }

  return nullptr;
}

//------------------------------------------------------------------------------
//
const bool DetectionTaskManager::IsAnotherUserMedia(const std::string &mediaName) {
  std::lock_guard<std::mutex> guard(_list_access);
  for (const auto &execution : detection_tasks_) {
    if (execution->GetMediaName() == mediaName) {
      return true;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<MediaStreamer> DetectionTaskManager::GetMediaStreamer(
    const std::string &mediaName) {
  std::lock_guard<std::mutex> guard(_list_access);
  for (auto &acquisition : media_streamers_) {
    if (acquisition->GetMediaName() == mediaName) {
      return acquisition;
    }
  }
  return nullptr;
}

}  // namespace vision_server
