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
   * Return a pointer of acquisitionLoop or execution with that name if exist.
   * Can return nullptr
   */
  std::shared_ptr<DetectionTask> GetExecution(const std::string &execName);

  std::shared_ptr<MediaStreamer> GetAcquisitionLoop(
      const std::string &mediaName);

  /**
   * Return true if another execution use the media
   */
  const bool IsAnotherUserMedia(const std::string &mediaName);

  /**
   * Add/remove MediaStreamer from the list.
   */
  void AddAcquisitionLoop(std::shared_ptr<MediaStreamer> ptr);

  void RemoveAcquisitionLoop(std::shared_ptr<MediaStreamer> ptr);

  /**
   * Add/remove DetectionTask from the list.
   */
  void AddExecution(std::shared_ptr<DetectionTask> ptr);

  void RemoveExecution(std::shared_ptr<DetectionTask> ptr);

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
  std::vector<std::shared_ptr<DetectionTask>> executions_;

  std::vector<std::shared_ptr<MediaStreamer>> acquisition_loop_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void VisionServer::AddAcquisitionLoop(
    std::shared_ptr<MediaStreamer> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  acquisition_loop_.push_back(ptr);
}

//------------------------------------------------------------------------------
//
inline void VisionServer::RemoveAcquisitionLoop(
    std::shared_ptr<MediaStreamer> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  auto acquisition = acquisition_loop_.begin();
  auto vec_end = acquisition_loop_.end();
  for (; acquisition != vec_end; ++acquisition) {
    if (*acquisition == ptr) {
      acquisition_loop_.erase(acquisition);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void VisionServer::AddExecution(std::shared_ptr<DetectionTask> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  executions_.push_back(ptr);
}

//------------------------------------------------------------------------------
//
inline void VisionServer::RemoveExecution(std::shared_ptr<DetectionTask> ptr) {
  std::lock_guard<std::mutex> guard(_list_access);
  auto execution = executions_.begin();
  auto vec_end = executions_.end();
  for (; execution != vec_end; ++execution) {
    if (*execution == ptr) {
      executions_.erase(execution);
    }
  }
}

}  // namespace vision_server

#endif  // VISION_SERVER_DETECTION_TASK_MANAGER_H_