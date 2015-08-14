/**
 * \file	VisionServer.cpp
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	24/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "server/vision_server.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
VisionServer::VisionServer(ROSManager ros_manager)
    : ROSCallbackManager(),
      _ros_manager(ros_manager),
      _camera_manager(ros_manager),
      _filterchain_manager(ros_manager) {
  auto base_node_name = std::string{VISION_NODE_NAME};

  RegisterService<vision_server_execute_cmd::Request,
                  vision_server_execute_cmd::Response>(
      base_node_name + "vision_server_execute_cmd",
      &VisionServer::CallbackExecutionCMD, *this);

  RegisterService<vision_server_get_information_list::Request,
                  vision_server_get_information_list::Response>(
      base_node_name + "vision_server_get_information_list",
      &VisionServer::CallbackInfoListCMD, *this);

  _list_access.Create();
};

//------------------------------------------------------------------------------
//
VisionServer::~VisionServer() {
  ROS_INFO_NAMED("[VISION_SERVER]", "Closing vision server.");
  acquisition_loop.clear();
  executions.clear();
  _list_access.Destroy();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
const Execution::Ptr VisionServer::GetExecution(const std::string &execName) {
  CLMutex::Guard guard(_list_access);
  for (const auto &execution : executions) {
    if (execution->GetExecName().find(execName.c_str()) != std::string::npos) {
      return execution;
    }
  }

  return nullptr;
}

//------------------------------------------------------------------------------
//
const bool VisionServer::IsAnotherUserMedia(const std::string &mediaName) {
  CLMutex::Guard guard(_list_access);
  for (const auto &execution : executions) {
    if (execution->GetMediaName() == mediaName) {
      return true;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
const AcquisitionLoop::Ptr VisionServer::GetAcquisitionLoop(
    const std::string &mediaName) {
  CLMutex::Guard guard(_list_access);
  for (auto &acquisition : acquisition_loop) {
    if (acquisition->GetMediaID().GetName() == mediaName) {
      return acquisition;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackExecutionCMD(
    vision_server_execute_cmd::Request &rqst,
    vision_server_execute_cmd::Response &rep) {
  // KILLL EVERYTHING!
  if (rqst.cmd == 3) {
    std::vector<std::string> media_names;
    for (auto &tmp : executions) {
      tmp->StopExec();
      // Security
      CLTimer::Delay(20);
    }
    executions.clear();
    for (auto &tmp : acquisition_loop) {
      _camera_manager.StreammingCmd(vision_server::CameraManager::STOP,
                                    tmp->GetMediaID().GetName(), tmp);
      // Security
      CLTimer::Delay(20);
    }
    acquisition_loop.clear();
  } else if (rqst.cmd == rqst.START) {
    std::cout << std::endl
              << std::endl;
    ROS_INFO_NAMED("[VISION_SERVER]",
                   "Stopping execution %s on %s with filterchain %s",
                   rqst.node_name.c_str(), rqst.media_name.c_str(),
                   rqst.filterchain_name.c_str());

    Execution::Ptr exec = GetExecution(rqst.node_name);
    if (exec.IsNotNull()) {
      ROS_WARN_NAMED("[VISION SERVER]",
                     " Execution of that name already exist");
      rep.response = exec->GetExecName();
    } else {
      AcquisitionLoop::Ptr acquiPtr = GetAcquisitionLoop(rqst.media_name);
      // Please change back to null string in production env

      // No acquisition loop running with this media.
      // Try to start one
      if (acquiPtr.IsNull()) {
        _camera_manager.StreammingCmd(CameraManager::START, rqst.media_name,
                                      acquiPtr);
        // Important to be here so we can push back ONLY if newly
        // started
        if (acquiPtr.IsNotNull()) AddAcquisitionLoop(acquiPtr);
      }
      // The media was found, and the streaming is on.
      if (acquiPtr.IsNotNull()) {
        // par défaut, ajouter le code pour prendre en compte la filter
        // chain
        // passée en paramètre
        Filterchain *filterchain = nullptr;
        filterchain = _filterchain_manager.InstanciateFilterchain(
            std::string(VISION_NODE_NAME) + rqst.node_name,
            rqst.filterchain_name);

        exec = new Execution(acquiPtr, filterchain, rqst.node_name);
        if (exec.IsNotNull()) {
          exec->StartExec();
          AddExecution(exec);
          rep.response = exec->GetExecName();
        }
      }
    }
  } else if (rqst.cmd == rqst.STOP) {
    std::cout << std::endl
              << std::endl;
    ROS_INFO_NAMED("[VISION_SERVER]",
                   "Stopping execution %s on %s with filterchain %s",
                   rqst.node_name.c_str(), rqst.media_name.c_str(),
                   rqst.filterchain_name.c_str());

    vision_server::Execution::Ptr exec = GetExecution(rqst.node_name);

    if (exec.IsNotNull()) {
      // Stop Exec.
      exec->StopExec();

      // Remove it from the list
      RemoveExecution(exec);

      // Check if another user. If no, stop the acquisition loop.
      if (!IsAnotherUserMedia(exec->GetMediaName())) {
        vision_server::AcquisitionLoop::Ptr aquiPtr =
            GetAcquisitionLoop(exec->GetMediaName());
        _camera_manager.StreammingCmd(vision_server::CameraManager::STOP,
                                      exec->GetID().GetName(), aquiPtr);
        RemoveAcquisitionLoop(aquiPtr);
      }
      // Should never happen... null filterchain...
      const Filterchain *fc = exec->getFilterChain();
      if (fc != nullptr) {
        _filterchain_manager.CloseFilterchain(exec->GetExecName(),
                                              fc->GetName());
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool vision_server::VisionServer::CallbackInfoListCMD(
    vision_server_get_information_list::Request &rqst,
    vision_server_get_information_list::Response &rep) {
  // Seems like we cannot do a switch case
  // since rqst.EXEC contains a . so cannot be evaluated?
  // Weird but hey, doesn't change much...

  if (rqst.cmd == rqst.EXEC) {
    rep.list = GetExecutionsList();
    // rep.list =
    // "exec1;exec2;exec3;exec4;exec5;exec6;exec7;exec8;exec9;exec10;exec11;exec12;exec13;exec14;exec15;";
    // //pour tests
  } else if (rqst.cmd == rqst.FILTERCHAIN) {
    rep.list = GetFilterChainList();
    // rep.list = "chain1;chain2;chain3;"; //pour tests
  } else if (rqst.cmd == rqst.FILTERS) {
    rep.list = GetFilterList();
    // rep.list = "cmdfiltre1;cmdfiltre2;"; //pour tests
  } else if (rqst.cmd == rqst.MEDIA) {
    rep.list = GetMediaList();
    // rep.list = "cmdfiltre1;cmdfiltre2;"; //pour tests
  }

  return true;
}

//------------------------------------------------------------------------------
//
std::string VisionServer::GetExecutionsList() {
  CLMutex::Guard guard(_list_access);
  auto execution_list_str = std::string{};
  for (const auto &execution : executions) {
    execution_list_str += execution->GetExecName() + ";";
  }

  return execution_list_str;
}

//------------------------------------------------------------------------------
//
std::string VisionServer::GetMediaList() {
  CLMutex::Guard guard(_list_access);
  auto camera_vec = _camera_manager.GetCameraList();
  auto camera_list_str = std::string{};
  for (const auto &camera : camera_vec) {
    camera_list_str += camera.GetName() + ";";
  }

  return camera_list_str;
}

//------------------------------------------------------------------------------
//
std::string VisionServer::GetFilterChainList() {
  auto filterchain_vec = _filterchain_manager.GetAvailableFilterchains();
  auto filterchain_str = std::string{};
  for (const auto &filterchain : filterchain_vec) {
    filterchain_str += filterchain + ";";
  }

  return filterchain_str;
}

//------------------------------------------------------------------------------
//
std::string VisionServer::GetFilterList() {
  return vision_filter::FilterFactory::GetFilterList();
}

}  // namespace vision_server
