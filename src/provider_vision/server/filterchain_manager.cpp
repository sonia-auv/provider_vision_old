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
#include <provider_vision/utils/pugixml.h>
#include "provider_vision/server/filterchain_manager.h"

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
  auto base_node_name = std::string{kRosNodeName};

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

}  // namespace vision_server
