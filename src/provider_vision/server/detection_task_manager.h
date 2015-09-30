/**
 * \file	detection_task_manager.h
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \author  Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_DETECTION_TASK_MANAGER_H_
#define PROVIDER_VISION_DETECTION_TASK_MANAGER_H_

#include <string>
#include <memory>
#include <mutex>

#include "provider_vision/proc/detection_task.h"

namespace vision_server {

class DetectionTaskManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DetectionTaskManager>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit DetectionTaskManager();

  ~DetectionTaskManager();

  //==========================================================================
  // P U B L I C   M E T H O D S

  std::vector<DetectionTask::Ptr> GetAllDetectionTasks() const
      noexcept;

  DetectionTask::Ptr GetDetectionTask(
      const std::string &execution_name) const;

  DetectionTask::Ptr CreateDetectionTask(
      std::shared_ptr<ros::NodeHandle> node_handle,
      MediaStreamer::Ptr acquisition_loop,
      Filterchain::Ptr filterchain,
      const std::string &execution_name) noexcept;

  void StopDetectionTask(const std::string &execution_name) noexcept;

  void StopDetectionTask(DetectionTask::Ptr) noexcept;

  std::vector<MediaStreamer::Ptr> GetAllMediaStreamers() const
      noexcept;

  MediaStreamer::Ptr GetMediaStreamer(
      const std::string &execution_name, const std::string &media_name) const
      noexcept;

  Filterchain::Ptr GetRunningFilterchains(
      const std::string &execution_name) const noexcept;

  std::vector<Filterchain::Ptr> GetAllRunningFilterchains() const noexcept;

  bool IsMediaUsed(const std::string &media_name);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::vector<DetectionTask::Ptr> detection_tasks_;
};

}  // namespace vision_server

#endif  // PROVIDER_VISION_DETECTION_TASK_MANAGER_H_
