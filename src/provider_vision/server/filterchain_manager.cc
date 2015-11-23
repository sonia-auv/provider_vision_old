/**
 * \file	filterchain_manager.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
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

namespace provider_vision {

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
std::vector<std::string> FilterchainManager::GetAllFilterchainName() const
    noexcept {
  auto availableFilterchains = std::vector<std::string>{};
  std::stringstream ss;
  ss << kConfigPath << "filterchain/";
  if (auto dir = opendir(ss.str().c_str())) {
    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
      auto filename = std::string{ent->d_name};
      if (filename.length() > kFilterchainExt.length() &&
          filename.substr(filename.length() - kFilterchainExt.length()) == kFilterchainExt) {
        filename.replace(filename.end() - kFilterchainExt.length(), filename.end(), "");
        availableFilterchains.push_back(filename);
      }
    }
    closedir(dir);
  }
  return availableFilterchains;
}

//------------------------------------------------------------------------------
//
void FilterchainManager::CreateFilterchain(const std::string &filterchain) {
  if (!FilterchainExists(filterchain)) {
    pugi::xml_document doc;
    doc.append_child("Filterchain");
    auto save_path = kConfigPath + filterchain + kFilterchainExt;
    doc.save_file(save_path.c_str());
  }
}

//------------------------------------------------------------------------------
//
void FilterchainManager::DeleteFilterchain(const std::string &filterchain) {
  remove(GetFilterchainPath(filterchain).c_str());
}

//------------------------------------------------------------------------------
//
bool FilterchainManager::FilterchainExists(const std::string &filterchain) {
  for (const auto &existing_filterchain : GetAllFilterchainName()) {
    if (filterchain == existing_filterchain) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
Filterchain::Ptr FilterchainManager::StartFilterchain(
    const std::string &executionName, const std::string &filterchainName) {
  if (FilterchainExists(filterchainName)) {
    Filterchain::Ptr filterchain =
        std::make_shared<Filterchain>(filterchainName, executionName);
    running_filterchains_.push_back(filterchain);
    return filterchain;
  }
  throw std::invalid_argument("Could not find the given filterchain");
}

//------------------------------------------------------------------------------
//
void FilterchainManager::StopFilterchain(const std::string &executionName,
                                         const std::string &filterchainName) {
  auto filterchain = running_filterchains_.begin();
  const auto &last_filterchain = running_filterchains_.end();
  for (; filterchain != last_filterchain; ++filterchain) {
    if ((*filterchain)->GetName().compare(filterchainName) == 0) {
      running_filterchains_.erase(filterchain);
    }
  }
}

//------------------------------------------------------------------------------
//
void FilterchainManager::SaveFilterchain(
    const std::string &executionName,
    const std::string &filterchainName) const {
  auto filterchain = GetRunningFilterchain(executionName);
  if (filterchain) {
    filterchain->Serialize();
  }
}

//------------------------------------------------------------------------------
//
Filterchain::Ptr FilterchainManager::GetRunningFilterchain(
    const std::string &execution) const noexcept {
  for (const auto &filterchain : running_filterchains_) {
    if (filterchain->GetName() == execution) {
      return filterchain;
    }
  }
  return nullptr;
}

}  // namespace provider_vision
