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

#include "provider_vision/server/vision_server.h"

namespace vision_server {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
VisionServer::VisionServer(std::shared_ptr<ros::NodeHandle> node_handle)
    : atlas::ServiceServerManager<VisionServer>(node_handle),
      node_handle_(node_handle),
      media_mgr_(),
      filterchain_mgr_() {
  auto base_node_name = std::string{kRosNodeName};

  RegisterService<vision_server_execute_cmd>(
      base_node_name + "vision_server_execute_cmd",
      &VisionServer::CallbackExecutionCMD, *this);

  RegisterService<vision_server_get_information_list>(
      base_node_name + "vision_server_get_information_list",
      &VisionServer::CallbackInfoListCMD, *this);

  RegisterService<vision_server_get_media_param>(
      base_node_name + "vision_server_get_media_param_list",
      &VisionServer::CallbackGetCMD, *this);

  RegisterService<vision_server_set_media_param>(
      base_node_name + "vision_server_set_media_param_list",
      &VisionServer::CallbackSetCMD, *this);

  RegisterService<vision_server_copy_filterchain>(
      base_node_name + "vision_server_copy_filterchain",
      &VisionServer::CallbackCopyFc, *this);

  RegisterService<vision_server_get_filterchain_filter_all_param>(
      base_node_name + "vision_server_get_filterchain_filter_all_param",
      &VisionServer::CallbackGetFilterAllParam, *this);

  RegisterService<vision_server_get_filterchain_filter_param>(
      base_node_name + "vision_server_get_filterchain_filter_param",
      &VisionServer::CallbackGetFilterParam, *this);

  RegisterService<vision_server_set_filterchain_filter_param>(
      base_node_name + "vision_server_set_filterchain_filter_param",
      &VisionServer::CallbackSetFilterParam, *this);

  RegisterService<vision_server_get_filterchain_filter>(
      base_node_name + "vision_server_get_filterchain_filter",
      &VisionServer::CallbackGetFilter, *this);

  RegisterService<vision_server_manage_filterchain_filter>(
      base_node_name + "vision_server_manage_filterchain_filter",
      &VisionServer::CallbackManageFilter, *this);

  RegisterService<vision_server_manage_filterchain>(
      base_node_name + "vision_server_manage_filterchain",
      &VisionServer::CallbackManageFc, *this);

  RegisterService<vision_server_save_filterchain>(
      base_node_name + "vision_server_save_filterchain",
      &VisionServer::CallbackSaveFc, *this);

  RegisterService<vision_server_set_filterchain_filter_order>(
      base_node_name + "vision_server_set_filterchain_filter_order",
      &VisionServer::CallbackSetFcOrder, *this);

  RegisterService<vision_server_get_filterchain_from_execution>(
      base_node_name + "vision_server_get_filterchain_from_execution",
      &VisionServer::CallbackGetFcFromExec, *this);

  RegisterService<vision_server_get_media_from_execution>(
      base_node_name + "vision_server_get_media_from_execution",
      &VisionServer::CallbackGetMediaFromExec, *this);

  RegisterService<vision_server_set_filterchain_filter_observer>(
      base_node_name + "vision_server_set_filterchain_filter_observer",
      &VisionServer::CallbackSetObserver, *this);
};

//------------------------------------------------------------------------------
//
VisionServer::~VisionServer() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackExecutionCMD(
    vision_server_execute_cmd::Request &rqst,
    vision_server_execute_cmd::Response &rep) {
  // KILLL EVERYTHING!
  if (rqst.cmd == 3) {
    std::vector<std::string> media_names;
    for (auto &tmp : executions_) {
      tmp->StopExec();
      // Security
      atlas::MilliTimer::sleep(20);
    }
    executions_.clear();
    for (auto &tmp : acquisition_loop_) {
      media_mgr_.StreammingCmd(vision_server::VisionServer::STOP,
                               tmp->GetMediaID().GetName(), tmp);
      // Security
      atlas::MilliTimer::sleep(20);
    }
    acquisition_loop_.clear();
  } else if (rqst.cmd == rqst.START) {
    std::cout << std::endl << std::endl;
    ROS_INFO_NAMED("[PROVIDER_VISION]",
                   "Stopping execution %s on %s with filterchain %s",
                   rqst.node_name.c_str(), rqst.media_name.c_str(),
                   rqst.filterchain_name.c_str());

    DetectionTask::Ptr exec = GetExecution(rqst.node_name);
    if (exec.get() != nullptr) {
      ROS_WARN_NAMED("[VISION SERVER]",
                     " DetectionTask of that name already exist");
      rep.response = exec->GetName();
    } else {
      MediaStreamer::Ptr acquiPtr = GetAcquisitionLoop(rqst.media_name);
      // Please change back to null string in production env

      // No acquisition loop running with this media.
      // Try to start one
      if (acquiPtr.get() == nullptr) {
        media_mgr_.StreammingCmd(VisionServer::START, rqst.media_name,
                                 acquiPtr);
        // Important to be here so we can push back ONLY if newly
        // started
        if (acquiPtr.get() != nullptr) {
          AddAcquisitionLoop(acquiPtr);
        }
      }
      // The media was found, and the streaming is on.
      if (acquiPtr.get() != nullptr) {
        // par défaut, ajouter le code pour prendre en compte la filter
        // chain
        // passée en paramètre
        Filterchain::Ptr filterchain = nullptr;
        filterchain = filterchain_mgr_.InstanciateFilterchain(
            std::string(kRosNodeName) + rqst.node_name, rqst.filterchain_name);

        exec = std::make_shared<DetectionTask>(node_handle_, acquiPtr,
                                               filterchain, rqst.node_name);
        exec->start();
        AddExecution(exec);
        rep.response = exec->GetName();
      }
    }
  } else if (rqst.cmd == rqst.STOP) {
    std::cout << std::endl << std::endl;
    ROS_INFO_NAMED("[PROVIDER_VISION]",
                   "Stopping execution %s on %s with filterchain %s",
                   rqst.node_name.c_str(), rqst.media_name.c_str(),
                   rqst.filterchain_name.c_str());

    DetectionTask::Ptr exec = GetExecution(rqst.node_name);

    if (exec.get() != nullptr) {
      // Stop Exec.
      exec->StopExec();

      // Remove it from the list
      RemoveExecution(exec);

      // Check if another user. If no, stop the acquisition loop.
      if (!IsAnotherUserMedia(exec->GetMediaName())) {
        MediaStreamer::Ptr aquiPtr = GetAcquisitionLoop(exec->GetMediaName());
        media_mgr_.StreammingCmd(vision_server::VisionServer::STOP,
                                 exec->GetID().GetName(), aquiPtr);
        RemoveAcquisitionLoop(aquiPtr);
      }
      // Should never happen... null filterchain...
      const Filterchain::Ptr fc = exec->GetFilterchain();
      if (fc != nullptr) {
        filterchain_mgr_.CloseFilterchain(exec->GetName(), fc->GetName());
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
    rep.list = vision_filter::FilterFactory::GetFilterList();
    // rep.list = "cmdfiltre1;cmdfiltre2;"; //pour tests
  } else if (rqst.cmd == rqst.MEDIA) {
    rep.list = GetMediaList();
    // rep.list = "cmdfiltre1;cmdfiltre2;"; //pour tests
  }

  return true;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackGetCMD(
    vision_server_get_media_param::Request &rqst,
    vision_server_get_media_param::Response &rep) {
  try {
    rep.value = media_mgr_.GetCameraFeature(rqst.media_name, rqst.param_name);
    return true;
  } catch (std::invalid_argument) {
    return false;
  }
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackSetCMD(
    vision_server_set_media_param::Request &rqst,
    vision_server_set_media_param::Response &rep) {
  try {
    media_mgr_.SetCameraFeature(rqst.media_name, rqst.param_name, rqst.value);
  } catch (std::invalid_argument) {
    rep.success = rep.FAIL;
    return false;
  }

  rep.success = rep.SUCCESS;
  return true;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackCopyFc(
    vision_server_copy_filterchain::Request &rqst,
    vision_server_copy_filterchain::Response &rep) {
  std::ifstream src(
      filterchain_mgr_.GetFilterchainPath(rqst.filterchain_to_copy).c_str(),
      std::ios::binary);
  std::ofstream dst(
      filterchain_mgr_.GetFilterchainPath(rqst.filterchain_new_name).c_str(),
      std::ios::binary);
  dst << src.rdbuf();
  rep.success = true;
  return rep.success;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackGetFilterParam(
    vision_server_get_filterchain_filter_param::Request &rqst,
    vision_server_get_filterchain_filter_param::Response &rep) {
  rep.list = "";

  std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
  Filterchain::Ptr filterchain =
      filterchain_mgr_.GetRunningFilterchain(execution_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetFilterParam(rqst.filter, rqst.parameter);
    return true;
  }
  ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackGetFilterAllParam(
    vision_server_get_filterchain_filter_all_param::Request &rqst,
    vision_server_get_filterchain_filter_all_param::Response &rep) {
  rep.list = "";

  std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
  Filterchain::Ptr filterchain =
      filterchain_mgr_.GetRunningFilterchain(execution_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetFilterAllParam(rqst.filter);
    return true;
  }
  ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackSetFilterParam(
    vision_server_set_filterchain_filter_param::Request &rqst,
    vision_server_set_filterchain_filter_param::Response &rep) {
  rep.success = rep.FAIL;

  std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
  Filterchain::Ptr filterchain =
      filterchain_mgr_.GetRunningFilterchain(execution_name);

  if (filterchain != nullptr) {
    filterchain->SetFilterParam(rqst.filter, rqst.parameter, rqst.value);
    rep.success = rep.SUCCESS;
    return true;
  }
  ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackGetFilter(
    vision_server_get_filterchain_filter::Request &rqst,
    vision_server_get_filterchain_filter::Response &rep) {
  rep.list = "";

  std::string execution_name(rqst.exec_name),
      filterchain_name(rqst.filterchain);
  Filterchain::Ptr filterchain =
      filterchain_mgr_.GetRunningFilterchain(execution_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetFilterList();
    return true;
  }

  std::string log_txt = "DetectionTask " + execution_name +
                        " does not exist or does not use this filterchain: " +
                        filterchain_name + " on get filter's param request.";
  ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filter's param request.",
      execution_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackSetObserver(
    vision_server_set_filterchain_filter_observer::Request &rqst,
    vision_server_set_filterchain_filter_observer::Response &rep) {
  // For now ignoring filterchain name, but when we will have multiple,
  // we will have to check the name and find the good filterchain
  Filterchain::Ptr filterchain =
      filterchain_mgr_.GetRunningFilterchain(rqst.execution);

  if (filterchain != nullptr) {
    rep.result = rep.SUCCESS;
    filterchain->SetObserver(rqst.filter);
    return true;
  }

  rep.result = rep.FAIL;
  ROS_INFO(
      "DetectionTask %s does not exist or does not use this "
      "filterchain: %s on get filters request",
      rqst.execution.c_str(), rqst.filterchain.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackManageFilter(
    vision_server_manage_filterchain_filter::Request &rqst,
    vision_server_manage_filterchain_filter::Response &rep) {
  const auto &filterchain =
      filterchain_mgr_.GetRunningFilterchain(rqst.exec_name);
  rep.success = 1;
  if (filterchain != nullptr) {
    if (rqst.cmd == rqst.ADD) {
      filterchain->AddFilter(rqst.filter);
    } else if (rqst.cmd == rqst.DELETE) {
      filterchain->RemoveFilter(
          filterchain->GetFilterIndexFromUIName(rqst.filter));
    }
  } else {
    rep.success = 0;
  }

  return rep.success;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackManageFc(
    vision_server_manage_filterchain::Request &rqst,
    vision_server_manage_filterchain::Response &rep) {
  std::string filterchain_name(rqst.filterchain);
  bool response = true;
  if (rqst.cmd == rqst.ADD) {
    if (filterchain_mgr_.CreateFilterchain(filterchain_name)) {
      rep.success = rep.SUCCESS;

    } else {
      rep.success = rep.FAIL;
    }
  } else if (rqst.cmd == rqst.DELETE) {
    filterchain_mgr_.DeleteFilterchain(filterchain_name);
  }
  return response;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackSaveFc(
    vision_server_save_filterchain::Request &rqst,
    vision_server_save_filterchain::Response &rep) {
  std::string execution_name(rqst.exec_name);
  std::string filterchain_name(rqst.filterchain);
  if (rqst.cmd == rqst.SAVE) {
    if (filterchain_mgr_.SaveFilterchain(execution_name, filterchain_name))
      rep.success = rep.SUCCESS;
    else
      rep.success = rep.FAIL;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackSetFcOrder(
    vision_server_set_filterchain_filter_order::Request &rqst,
    vision_server_set_filterchain_filter_order::Response &rep) {
  ROS_INFO("Call to vision_server_set_filterchain_filter_order.");

  rep.success = rep.SUCCESS;
  for (const auto &filterchain :
       filterchain_mgr_.GetRunningFilterchain(rqst.exec_name)) {
    if (rqst.cmd == rqst.UP) {
      filterchain->MoveFilterUp(rqst.filter_index);
    } else if (rqst.cmd == rqst.DOWN) {
      filterchain->MoveFilterDown(rqst.filter_index);
    } else {
      ROS_INFO("Filter index provided was invalid");
      rep.success = rep.FAIL;
    }
  }

  return rep.success;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackGetFcFromExec(
    vision_server_get_filterchain_from_execution::Request &rqst,
    vision_server_get_filterchain_from_execution::Response &rep) {
  ROS_INFO("Call to vision_server_get_filterchain_from_execution.");
  std::string execution_name(rqst.exec_name);
  Filterchain::Ptr filterchain =
      filterchain_mgr_.GetRunningFilterchain(execution_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetName();
    return true;
  }
  ROS_INFO(
      "DetectionTask %s does not exist or does not use this filterchain "
      "on get filters request.",
      execution_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool VisionServer::CallbackGetMediaFromExec(
    vision_server_get_media_from_execution::Request &rqst,
    vision_server_get_media_from_execution::Response &rep) {
  ROS_INFO("Call to vision_server_get_media_from_execution.");
  auto response = "media_" + rqst.exec_name;
  rep.list = response;
  return true;
}

}  // namespace vision_server
