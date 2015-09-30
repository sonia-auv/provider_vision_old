/**
 * \file	FilterchainManager.h
 * \author  Frédéric-Simon Mimeault <frederic.simon.mimeault@gmail.com>
 * \author	Thomas Fuhrmann <tomesman@gmail.com>
 * \author  Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	24/01/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_FILTERCHAIN_MANAGER_H_
#define PROVIDER_VISION_FILTERCHAIN_MANAGER_H_

#include <functional>
#include <lib_atlas/ros/service_server_manager.h>
#include <lib_vision/filter.h>
#include "provider_vision/config.h"
#include "provider_vision/proc/filterchain.h"

namespace vision_server {

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

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit FilterchainManager();

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
  std::vector<std::string> GetAvailableFilterchains();

  /**
   * If the filterchain exists, this method will create
   * an ins
   *
   * \param executionName std::string
   * \param filterchainName std::string
   * \return Filterchain*
   */
  Filterchain::Ptr InstanciateFilterchain(
      std::string executionName, std::string filterchainName);

  /**
   * Get all available filterchains on the system.
   *
   * \param executionName std::string
   * \param filterchainName std::string
   * \return Filterchain*
   */
  bool CloseFilterchain(std::string executionName, std::string filterchainName);

  /**
   * Get all available filterchains on the system
   *
   * \param executionName std::string
   * \param filterchainName std::string
   * \return Filterchain*
   */
  bool SaveFilterchain(std::string executionName, std::string filterchainName);

  /**
   * Find an existing filterchain
   *
   * \param execution_name string
   * \param filterchainName string
   * \return filterchain Filterchain*
   */
  Filterchain::Ptr GetRunningFilterchain(
      const std::string &execution);

  /**
   * If the does not filterchain exists, create it.
   * \param filterchain The name of the filterchain to create.
   * \return Either if the filterchain was added or not.
   */
  bool CreateFilterchain(const std::string &filterchain);

  /**
   * If the filterchain exists, delete it.
   *
   * \param filterchain The name of the filterchain to delete.
   * \return Either if the filterchain was delete or not.
   */
  bool DeleteFilterchain(const std::string &filterchain);

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

  /**
   * Display all the available filterchains on the standard output.
   */
  void ListAvailableFilterchains();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * CameraConfig _cam_config
   */
  const char *FILTERCHAIN_MANAGER_TAG;

  /**
   * List of current instances of filterchains
   */
  std::vector<Filterchain::Ptr> _runningFilterchains;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline std::string FilterchainManager::GetFilterchainPath(
    const std::string &filterchain) const {
  return kConfigPath + filterchain + kFilterchainExt;
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_FILTERCHAIN_MANAGER_H_
