/**
 * \file	filterchain_manager.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_SERVER_FILTERCHAIN_MANAGER_H_
#define PROVIDER_VISION_SERVER_FILTERCHAIN_MANAGER_H_

#include <lib_atlas/ros/service_server_manager.h>
#include <provider_vision/filters/filter.h>
#include <functional>
#include <string>
#include <vector>
#include "provider_vision/config.h"
#include "provider_vision/server/filterchain.h"

namespace provider_vision {

/**
 * This class is the core module that stores and manages every FilterChains
 * within the vision server. Its job is to keep track of all changes occuring
 * to each Filterchain in addition to being charged of opening and closing them.
 * Also offers ROS services to allow filterchain managing.
 */
class FilterchainManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FilterchainManager>;

  static const std::string FILTERCHAIN_MANAGER_TAG;

  //==========================================================================
  // P U B L I C   C / D T O R S

  FilterchainManager();

  ~FilterchainManager();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Get all available filterchains on the system.
   *
   * \param execution_name : string
   * \param filterchain : string
   * \param filter : string
   * \return vector<std::string>
   */
  std::vector<std::string> GetAllFilterchainName() const noexcept;

  /**
   * Get the list of all running filterchains
   */
  const std::vector<Filterchain::Ptr> &GetRunningFilterchains() const;

  /**
   * If the filterchain exists, this method will create
   * an ins
   *
   * \param filterchainName std::string
   * \return Filterchain*
   */
  Filterchain::Ptr InstanciateFilterchain(const std::string &filterchainName);

  /**
   * Pass through the list of all the filterchains and instanciate them
   *
   * \return The list of all filter chains
   */
  const std::vector<Filterchain::Ptr> &InstanciateAllFilterchains();

  /**
   * Get all available filterchains on the system.
   *
   * \param executionName std::string
   * \param filterchainName std::string
   * \return Filterchain*
   */
  void StopFilterchain(const Filterchain::Ptr &filterchain);

  /**
   * If the does not filterchain exists, create it.
   *
   * \param filterchain The name of the filterchain to create.
   * \return Either if the filterchain was added or not.
   */
  void CreateFilterchain(const std::string &filterchain);

  /**
   * If the filterchain exists, delete it.
   *
   * \param filterchain The name of the filterchain to delete.
   * \return Either if the filterchain was delete or not.
   */
  void EraseFilterchain(const std::string &filterchain);

  /**
   * Check if a filterchain exist or not.
   *
   * This will check on the list of the available filterchain provided by
   * GetAvailableFilterchain if the file exists or not.
   *
   * \param filterchain The name of the filterchain to check.
   * \return Either if the file exist or not
   */
  bool FilterchainExists(const std::string &filterchain);

  /**
   * With the constants defining the config directory path and the
   * extension, return the true path of a filterchain.
   */
  std::string GetFilterchainPath(const std::string &filterchain) const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * List of current instances of filterchains
   */
  std::vector<Filterchain::Ptr> running_filterchains_;
};

}  // namespace provider_vision

#endif  // PROVIDER_VISION_SERVER_FILTERCHAIN_MANAGER_H_
