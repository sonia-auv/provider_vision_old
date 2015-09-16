/**
 * \file	detection_task_manager.cpp
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

namespace vision_server {

//------------------------------------------------------------------------------
//
std::shared_ptr<DetectionTask> VisionServer::GetExecution(
    const std::string &execName) {
  std::lock_guard<std::mutex> guard(_list_access);
  for (const auto &execution : executions_) {
    if (execution->GetExecName().find(execName.c_str()) != std::string::npos) {
      return execution;
    }
  }

  return nullptr;
}

//------------------------------------------------------------------------------
//
const bool VisionServer::IsAnotherUserMedia(const std::string &mediaName) {
  std::lock_guard<std::mutex> guard(_list_access);
  for (const auto &execution : executions_) {
    if (execution->GetMediaName() == mediaName) {
      return true;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
std::shared_ptr<MediaStreamer> VisionServer::GetAcquisitionLoop(
    const std::string &mediaName) {
  std::lock_guard<std::mutex> guard(_list_access);
  for (auto &acquisition : acquisition_loop_) {
    if (acquisition->GetMediaID().GetName() == mediaName) {
      return acquisition;
    }
  }
  return nullptr;
}

}  // namespace vision_server
