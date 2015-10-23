/**
 * \file	detection_task_manager.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_SERVER_DETECTION_TASK_MANAGER_H_
#define PROVIDER_VISION_SERVER_DETECTION_TASK_MANAGER_H_

#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include "provider_vision/proc/detection_task.h"
#include "provider_vision/proc/filterchain.h"

namespace vision_server {

class DetectionTaskManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DetectionTaskManager>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  DetectionTaskManager();

  ~DetectionTaskManager();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void StartDetectionTask(MediaStreamer::Ptr media_streamer,
                          Filterchain::Ptr filterchain,
                          const std::string &execution_name) noexcept;

  void StopDetectionTask(const std::string &execution_name) noexcept;

  std::vector<std::string> GetAllDetectionTasksName() const noexcept;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  DetectionTask::Ptr GetDetectionTask(const std::string &execution_name) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::vector<DetectionTask::Ptr> detection_tasks_;
};

}  // namespace vision_server

#endif  // PROVIDER_VISION_DETECTION_TASK_MANAGER_H_
