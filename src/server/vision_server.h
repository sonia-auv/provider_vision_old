/**
 * \file	VisionServer.cpp
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	24/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_VISION_SERVER_H_
#define VISION_SERVER_VISION_SERVER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <CLMutex.h>
#include <HTSmartPtr.h>
#include <vision_server/vision_server_execute_cmd.h>
#include <vision_server/vision_server_get_information_list.h>
#include "media/media.h"
#include "config.h"
#include "ros/ros_manager.h"
#include "ros/ros_callback_manager.h"
#include "server/camera_manager.h"
#include "server/filterchain_manager.h"
#include "server/execution.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * Vision server is the main class of the system
 * It's job is to assemble and connect the pieces to create execution
 * It is the owner of the active Execution and AcquisitionLoop
 * It does not hold the filterchains, as it is the responsability of the
 * filterchain manager.
 * It gets the filterchains from the filterchain manager.
 *
 * The visionServer is also the service point for listing service (media, exec,
 * filterchain and filter)
 * and the start/stop execution.
 */
class VisionServer : public ROSCallbackManager<VisionServer> {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  VisionServer(ROSManager ros_manager);

  ~VisionServer();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Return a pointer of acquisitionLoop or execution with that name if exist.
   * Can return nullptr
   */
  const Execution::Ptr GetExecution(const std::string &execName);

  const AcquisitionLoop::Ptr GetAcquisitionLoop(const std::string &mediaName);

  /**
   * Return true if another execution use the media
   */
  const bool IsAnotherUserMedia(const std::string &mediaName);

  /**
   * Add/remove AcquisitionLoop from the list.
   */
  void AddAcquisitionLoop(const AcquisitionLoop::Ptr &ptr);

  void RemoveAcquisitionLoop(const AcquisitionLoop::Ptr &ptr);

  /**
   * Add/remove Execution from the list.
   */
  void AddExecution(const Execution::Ptr &ptr);

  void RemoveExecution(const Execution::Ptr &ptr);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  bool CallbackExecutionCMD(
      vision_server::vision_server_execute_cmd::Request &rqst,
      vision_server::vision_server_execute_cmd::Response &rep);

  bool CallbackInfoListCMD(
      vision_server::vision_server_get_information_list::Request &rqst,
      vision_server::vision_server_get_information_list::Response &rep);

  /**
   * Return a string of each execution separated by the current
   * COMPONENT_SEPARATOR given a list of the executions.
   */
  std::string GetExecutionsList();

  /**
   * Return a string of each medias separated by the current
   * COMPONENT_SEPARATOR given a list of the medias.
   */
  std::string GetMediaList();

  /**
   * Return a filterchains of each execution separated by the current
   * COMPONENT_SEPARATOR given a list of the filterchains
   */
  std::string GetFilterChainList();

  /**
   * Return a string of each filters separated by the current
   * COMPONENT_SEPARATOR given a list of the filters.
   */
  std::string GetFilterList();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ROSManager _ros_manager;

  CameraManager _camera_manager;

  FilterchainManager _filterchain_manager;

  /**
   * Protect the access of the vectors
   */
  CLMutex _list_access;

  /**
   * Remember what exist.
   */
  std::vector<Execution::Ptr> executions;

  std::vector<AcquisitionLoop::Ptr> acquisition_loop;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void VisionServer::AddAcquisitionLoop(const AcquisitionLoop::Ptr &ptr) {
  CLMutex::Guard guard(_list_access);
  acquisition_loop.push_back(ptr);
}

//------------------------------------------------------------------------------
//
inline void VisionServer::RemoveAcquisitionLoop(
    const AcquisitionLoop::Ptr &ptr) {
  CLMutex::Guard guard(_list_access);
  auto acquisition = acquisition_loop.begin();
  auto vec_end = acquisition_loop.end();
  for (; acquisition != vec_end; ++acquisition) {
    if (*acquisition == ptr) {
      acquisition_loop.erase(acquisition);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void VisionServer::AddExecution(const Execution::Ptr &ptr) {
  CLMutex::Guard guard(_list_access);
  executions.push_back(ptr);
}

//------------------------------------------------------------------------------
//
inline void VisionServer::RemoveExecution(const Execution::Ptr &ptr) {
  CLMutex::Guard guard(_list_access);
  auto execution = executions.begin();
  auto vec_end = executions.end();
  for (; execution != vec_end; ++execution) {
    if (*execution == ptr) {
      executions.erase(execution);
    }
  }
}

}  // namespace vision_server

#endif  // VISION_SERVER_VISION_SERVER_H_