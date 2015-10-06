/**
 * \file	FilterchainManager.cpp
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <dirent.h>
#include <provider_vision/utils/pugixml.h>
#include "provider_vision/server/filterchain_manager.h"

namespace vision_server {

const std::string FilterchainManager::FILTERCHAIN_MANAGER_TAG =
    "FILTERCHAIN_MANAGER";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
FilterchainManager::FilterchainManager(){};

//------------------------------------------------------------------------------
//
FilterchainManager::~FilterchainManager() {}

//==============================================================================
// M E T H O D   S E C T I O N
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
Filterchain::Ptr FilterchainManager::InstanciateFilterchain(
    std::string executionName, std::string filterchainName) {
  if (FilterchainExists(filterchainName)) {
    Filterchain::Ptr filterchain =
        std::make_shared<Filterchain>(filterchainName, executionName);
    _runningFilterchains.push_back(filterchain);
    return filterchain;
  }
  ROS_ERROR_NAMED(FILTERCHAIN_MANAGER_TAG.c_str(),
                  "Could not find the filterchain: %s",
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
    if ((*filterchain)->GetName().compare(filterchainName) == 0) {
      _runningFilterchains.erase(filterchain);
    }
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::SaveFilterchain(std::string executionName,
                                         std::string filterchainName) {
  auto filterchain = GetRunningFilterchain(executionName);
  if (filterchain) {
    filterchain->Serialize();
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
Filterchain::Ptr FilterchainManager::GetRunningFilterchain(
    const std::string &execution) {
  for (const auto &filterchain : _runningFilterchains) {
    if (filterchain->GetName() == execution) {
      return filterchain;
    }
  }
  return nullptr;
}

}  // namespace vision_server
