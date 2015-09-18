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

#ifndef VISION_SERVER_DETECTION_TASK_MANAGER_H_
#define VISION_SERVER_DETECTION_TASK_MANAGER_H_

#include <string>
#include <memory>
#include <mutex>

#include "provider_vision/proc/detection_task.h"

namespace vision_server {

class DetectionTaskManager {
 public:
  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit DetectionTaskManager();

  ~DetectionTaskManager();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Return a pointer of MediaStreamer or execution with that name if exist.
   * Can return nullptr
   */
  std::shared_ptr<DetectionTask> GetDetectionTask(const std::string &execName);


  std::shared_ptr<MediaStreamer> GetMediaStreamer(
      const std::string &mediaName);

  /**
   * Return true if another DetectionTask use the media
   */
  const bool IsAnotherUserMedia(const std::string &mediaName);

  /**
   * Add/remove MediaStreamer from the list.
   */
  void AddMediaStreamer(std::shared_ptr<MediaStreamer> ptr);

  void RemoveMediaStreamer(std::shared_ptr<MediaStreamer> ptr);

  /**
   * Add/remove DetectionTask from the list.
   */
  void AddDetectionTask(std::shared_ptr<DetectionTask> ptr);

  void RemoveDetectionTask(std::shared_ptr<DetectionTask> ptr);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
  * Protect the access of the vectors
  */
  mutable std::mutex _list_access;

  /**
   * Remember what exist.
   */
  std::vector<std::shared_ptr<DetectionTask>> detection_tasks_;

  std::vector<std::shared_ptr<MediaStreamer>> media_streamers_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void DetectionTaskManager::AddMediaStreamer(
    std::shared_ptr<MediaStreamer> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  media_streamers_.push_back(ptr);
}

//------------------------------------------------------------------------------
//
inline void DetectionTaskManager::RemoveMediaStreamer(
    std::shared_ptr<MediaStreamer> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  auto acquisition = media_streamers_.begin();
  auto vec_end = media_streamers_.end();
  for (; acquisition != vec_end; ++acquisition) {
    if (*acquisition == ptr) {
      media_streamers_.erase(acquisition);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void DetectionTaskManager::AddDetectionTask(std::shared_ptr<DetectionTask> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  detection_tasks_.push_back(ptr);
}

//------------------------------------------------------------------------------
//
inline void DetectionTaskManager::RemoveDetectionTask(std::shared_ptr<DetectionTask> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  auto execution = detection_tasks_.begin();
  auto vec_end = detection_tasks_.end();
  for (; execution != vec_end; ++execution) {
    if (*execution == ptr) {
      detection_tasks_.erase(execution);
    }
  }
}

}  // namespace vision_server

#endif  // VISION_SERVER_DETECTION_TASK_MANAGER_H_
