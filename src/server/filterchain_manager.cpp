/**
 * \file	FilterchainManager.cpp
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <assert.h>
#include <dirent.h>
#include <utils/pugixml.h>
#include "server/filterchain_manager.h"

namespace vision_server {

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
FilterchainManager::FilterchainManager(atlas::NodeHandlePtr node_handle)
    : atlas::ServiceServerManager<FilterchainManager>(node_handle),
      FILTERCHAIN_MANAGER_TAG("FILTERCHAIN_MANAGER") {
  assert(node_handle.get() != nullptr);

  std::cout << "Initialising FilterchainManager..." << std::endl;
  std::cout << "FilterchainList XML path : " << kConfigPath << std::endl;

  ListAvailableFilterchains();

  // création des node ROS pour les services de gestion des fc
  auto base_node_name = std::string{VISION_NODE_NAME};

  RegisterService<vision_server_copy_filterchain>(
      base_node_name + "vision_server_copy_filterchain",
      &FilterchainManager::CallbackCopyFc, *this);

  RegisterService<vision_server_get_filterchain_filter_all_param>(
      base_node_name + "vision_server_get_filterchain_filter_all_param",
      &FilterchainManager::CallbackGetFilterAllParam, *this);

  RegisterService<vision_server_get_filterchain_filter_param>(
      base_node_name + "vision_server_get_filterchain_filter_param",
      &FilterchainManager::CallbackGetFilterParam, *this);

  RegisterService<vision_server_set_filterchain_filter_param>(
      base_node_name + "vision_server_set_filterchain_filter_param",
      &FilterchainManager::CallbackSetFilterParam, *this);

  RegisterService<vision_server_get_filterchain_filter>(
      base_node_name + "vision_server_get_filterchain_filter",
      &FilterchainManager::CallbackGetFilter, *this);

  RegisterService<vision_server_manage_filterchain_filter>(
      base_node_name + "vision_server_manage_filterchain_filter",
      &FilterchainManager::CallbackManageFilter, *this);

  RegisterService<vision_server_manage_filterchain>(
      base_node_name + "vision_server_manage_filterchain",
      &FilterchainManager::CallbackManageFc, *this);

  RegisterService<vision_server_save_filterchain>(
      base_node_name + "vision_server_save_filterchain",
      &FilterchainManager::CallbackSaveFc, *this);

  RegisterService<vision_server_set_filterchain_filter_order>(
      base_node_name + "vision_server_set_filterchain_filter_order",
      &FilterchainManager::CallbackSetFcOrder, *this);

  RegisterService<vision_server_get_filterchain_from_execution>(
      base_node_name + "vision_server_get_filterchain_from_execution",
      &FilterchainManager::CallbackGetFcFromExec, *this);

  RegisterService<vision_server_get_media_from_execution>(
      base_node_name + "vision_server_get_media_from_execution",
      &FilterchainManager::CallbackGetMediaFromExec, *this);

  RegisterService<vision_server_set_filterchain_filter_observer>(
      base_node_name + "vision_server_set_filterchain_filter_observer",
      &FilterchainManager::CallbackSetObserver, *this);

  InstanciateFilterchain("test", "test_filter");
};

//------------------------------------------------------------------------------
//
FilterchainManager::~FilterchainManager() {
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG, "Closing filterchain manager.");
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void FilterchainManager::ListAvailableFilterchains() {
  std::cout << "Populating availableFilterchains..." << std::endl;

  auto availableFilterchains = GetAvailableFilterchains();
  std::cout << "Nb filterchains : " << availableFilterchains.size()
            << std::endl;

  int i = 1;
  for (const auto filterchain : availableFilterchains) {
    std::cout << "Filterchain " << i << " : " << filterchain << std::endl;
    ++i;
  }
}

//------------------------------------------------------------------------------
//
std::vector<std::string> FilterchainManager::GetAvailableFilterchains() {
  auto availableFilterchains = std::vector<std::string>{};

  if (auto dir = opendir(kConfigPath.c_str())) {
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
      auto filename = std::string{ent->d_name};
      if (filename.length() > 3 &&
          filename.substr(filename.length() - 3) == kFilterchainExt) {
        filename.replace(filename.end() - 3, filename.end(), "");
        availableFilterchains.push_back(filename);
      }
    };
    closedir(dir);
  }
  return availableFilterchains;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CreateFilterchain(const std::string &filterchain) {
  if (!FilterchainExists(filterchain)) {
    pugi::xml_document doc;
    doc.append_child("Filterchain");
    auto save_path = kConfigPath + filterchain + kFilterchainExt;
    doc.save_file(save_path.c_str());
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::DeleteFilterchain(const std::string &filterchain) {
  if (remove(GetFilterchainPath(filterchain).c_str())) {
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::FilterchainExists(const std::string &filterchain) {
  for (const auto &existing_filterchain : GetAvailableFilterchains()) {
    if (filterchain == existing_filterchain) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
Filterchain *FilterchainManager::InstanciateFilterchain(
    std::string executionName, std::string filterchainName) {
  if (FilterchainExists(filterchainName)) {
    const auto filterchain = new Filterchain(filterchainName, executionName);
    _runningFilterchains.push_back(filterchain);
    return filterchain;
  }
  ROS_ERROR_NAMED(FILTERCHAIN_MANAGER_TAG, "Could not find the filterchain: %s",
                  filterchainName.c_str());
  return nullptr;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CloseFilterchain(std::string executionName,
                                          std::string filterchainName) {
  auto filterchain = _runningFilterchains.begin();
  const auto last_filterchain = _runningFilterchains.end();
  for (; filterchain != last_filterchain; ++filterchain) {
    if (((*filterchain)->GetExecutionName() == executionName) &&
        ((*filterchain)->GetName() == filterchainName)) {
      _runningFilterchains.erase(filterchain);
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::SaveFilterchain(std::string executionName,
                                         std::string filterchainName) {
  auto filterchain = GetRunningFilterchain(executionName, filterchainName);
  if (filterchain) {
    filterchain->Serialize();
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
Filterchain *FilterchainManager::GetRunningFilterchain(
    std::string executionName, std::string filterchainName) {
  for (const auto &filterchain : _runningFilterchains) {
    if (filterchain->GetExecutionName() == executionName) {
      return filterchain;
    }
  }
  return nullptr;
}

//==============================================================================
// C A L L B A C K   R O S   S E R V I C E   S E C T I O N

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackCopyFc(
    vision_server_copy_filterchain::Request &rqst,
    vision_server_copy_filterchain::Response &rep) {
  std::ifstream src(GetFilterchainPath(rqst.filterchain_to_copy),
                    std::ios::binary);
  std::ofstream dst(GetFilterchainPath(rqst.filterchain_new_name),
                    std::ios::binary);
  dst << src.rdbuf();
  rep.success = true;
  return rep.success;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackGetFilterParam(
    vision_server_get_filterchain_filter_param::Request &rqst,
    vision_server_get_filterchain_filter_param::Response &rep) {
  rep.list = "";

  std::string exec_name(rqst.exec_name), filterchain_name(rqst.filterchain);
  Filterchain *filterchain = GetRunningFilterchain(exec_name, filterchain_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetFilterParam(rqst.filter, rqst.parameter);
    return true;
  }
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Execution %s does not exist or does not use this "
                 "filterchain: %s on get filter's param request.",
                 exec_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackGetFilterAllParam(
    vision_server_get_filterchain_filter_all_param::Request &rqst,
    vision_server_get_filterchain_filter_all_param::Response &rep) {
  rep.list = "";

  std::string exec_name(rqst.exec_name), filterchain_name(rqst.filterchain);
  Filterchain *filterchain = GetRunningFilterchain(exec_name, filterchain_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetFilterAllParam(rqst.filter);
    return true;
  }
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Execution %s does not exist or does not use this "
                 "filterchain: %s on get filter's param request.",
                 exec_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackSetFilterParam(
    vision_server_set_filterchain_filter_param::Request &rqst,
    vision_server_set_filterchain_filter_param::Response &rep) {
  rep.success = rep.FAIL;

  std::string exec_name(rqst.exec_name), filterchain_name(rqst.filterchain);
  Filterchain *filterchain = GetRunningFilterchain(exec_name, filterchain_name);

  if (filterchain != nullptr) {
    filterchain->SetFilterParam(rqst.filter, rqst.parameter, rqst.value);
    rep.success = rep.SUCCESS;
    return true;
  }
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Execution %s does not exist or does not use this "
                 "filterchain: %s on get filter's param request.",
                 exec_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackGetFilter(
    vision_server_get_filterchain_filter::Request &rqst,
    vision_server_get_filterchain_filter::Response &rep) {
  rep.list = "";

  std::string exec_name(rqst.exec_name), filterchain_name(rqst.filterchain);
  Filterchain *filterchain = GetRunningFilterchain(exec_name, filterchain_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetFilterList();
    return true;
  }

  std::string log_txt = "Execution " + exec_name +
                        " does not exist or does not use this filterchain: " +
                        filterchain_name + " on get filter's param request.";
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Execution %s does not exist or does not use this "
                 "filterchain: %s on get filter's param request.",
                 exec_name.c_str(), filterchain_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackSetObserver(
    vision_server_set_filterchain_filter_observer::Request &rqst,
    vision_server_set_filterchain_filter_observer::Response &rep) {
  // For now ignoring filterchain name, but when we will have multiple,
  // we will have to check the name and find the good filterchain
  Filterchain *filterchain =
      GetRunningFilterchain(rqst.execution, rqst.filterchain);

  if (filterchain != nullptr) {
    rep.result = rep.SUCCESS;
    filterchain->SetObserver(rqst.filter);
    return true;
  }

  rep.result = rep.FAIL;
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Execution %s does not exist or does not use this "
                 "filterchain: %s on get filters request",
                 rqst.execution.c_str(), rqst.filterchain.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackManageFilter(
    vision_server_manage_filterchain_filter::Request &rqst,
    vision_server_manage_filterchain_filter::Response &rep) {
  const auto &filterchain =
      GetRunningFilterchain(rqst.exec_name, rqst.filterchain);
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
bool FilterchainManager::CallbackManageFc(
    vision_server_manage_filterchain::Request &rqst,
    vision_server_manage_filterchain::Response &rep) {
  std::string filterchain_name(rqst.filterchain);
  bool response = true;
  if (rqst.cmd == rqst.ADD) {
    if (CreateFilterchain(filterchain_name)) {
      rep.success = rep.SUCCESS;

    } else {
      rep.success = rep.FAIL;
    }
  } else if (rqst.cmd == rqst.DELETE) {
    DeleteFilterchain(filterchain_name);
  }
  return response;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackSaveFc(
    vision_server_save_filterchain::Request &rqst,
    vision_server_save_filterchain::Response &rep) {
  std::string exec_name(rqst.exec_name);
  std::string filterchain_name(rqst.filterchain);
  if (rqst.cmd == rqst.SAVE) {
    if (SaveFilterchain(exec_name, filterchain_name))
      rep.success = rep.SUCCESS;
    else
      rep.success = rep.FAIL;
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackSetFcOrder(
    vision_server_set_filterchain_filter_order::Request &rqst,
    vision_server_set_filterchain_filter_order::Response &rep) {
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Call to vision_server_set_filterchain_filter_order.");

  rep.success = rep.SUCCESS;

  for (const auto &filterchain : _runningFilterchains) {
    if (filterchain->GetExecutionName() == rqst.exec_name) {
      if (rqst.cmd == rqst.UP) {
        filterchain->MoveFilterUp(rqst.filter_index);
      } else if (rqst.cmd == rqst.DOWN) {
        filterchain->MoveFilterDown(rqst.filter_index);
      } else {
        ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                       "Filter index provided was invalid");
        rep.success = rep.FAIL;
      }
      break;
    }
  }

  return rep.success;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackGetFcFromExec(
    vision_server_get_filterchain_from_execution::Request &rqst,
    vision_server_get_filterchain_from_execution::Response &rep) {
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Call to vision_server_get_filterchain_from_execution.");
  std::string exec_name(rqst.exec_name);
  Filterchain *filterchain = GetRunningFilterchain(exec_name);

  if (filterchain != nullptr) {
    rep.list = filterchain->GetName();
    return true;
  }
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Execution %s does not exist or does not use this filterchain "
                 "on get filters request.",
                 exec_name.c_str());
  return false;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::CallbackGetMediaFromExec(
    vision_server_get_media_from_execution::Request &rqst,
    vision_server_get_media_from_execution::Response &rep) {
  ROS_INFO_NAMED(FILTERCHAIN_MANAGER_TAG,
                 "Call to vision_server_get_media_from_execution.");
  const auto response = "media_" + rqst.exec_name;
  rep.list = response;
  return true;
}

}  // namespace vision_server
