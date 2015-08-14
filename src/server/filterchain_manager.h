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

#ifndef VISION_SERVER_FILTERCHAIN_MANAGER_H_
#define VISION_SERVER_FILTERCHAIN_MANAGER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <functional>
#include <CLList.h>
#include "vision_server/vision_server_copy_filterchain.h"
#include "vision_server/vision_server_manage_filterchain.h"
#include "vision_server/vision_server_get_filterchain_filter_param.h"
#include "vision_server/vision_server_set_filterchain_filter_param.h"
#include "vision_server/vision_server_get_filterchain_filter_all_param.h"
#include "vision_server/vision_server_get_filterchain_filter.h"
#include "vision_server/vision_server_manage_filterchain_filter.h"
#include "vision_server/vision_server_save_filterchain.h"
#include "vision_server/vision_server_set_filterchain_filter_order.h"
#include "vision_server/vision_server_get_filterchain_from_execution.h"
#include "vision_server/vision_server_get_media_from_execution.h"
#include "vision_server/vision_server_set_filterchain_filter_observer.h"
#include <lib_vision/filter.h>
#include "utils/constants.h"
#include "utils/camera_id.h"
#include "ros/ros_manager.h"

#include "ros/ros_callback_manager.h"
#include "server/filterchain.h"

namespace vision_server {

using namespace vision_server;

//==============================================================================
// C L A S S E S

/**
 * This class is the core module that stores and manages every FilterChains
 * within the vision server. Its job is to keep track of all changes occuring
 * to each Filterchain in addition to being charged of opening and closing them.
 * Also offers ROS services to allow filterchain managing.
 */
class FilterchainManager : public ROSCallbackManager<FilterchainManager> {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  FilterchainManager(ROSManager &manager);

  ~FilterchainManager();

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Get all available filterchains on the system.
   *
   * \param exec_name : string
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
  Filterchain *InstanciateFilterchain(std::string executionName,
                                      std::string filterchainName);

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
   * \param exec_name string
   * \param filterchainName string
   * \return filterchain Filterchain*
   */
  Filterchain *GetRunningFilterchain(std::string executionName,
                                     std::string filterchainName = "");

  /**
   * \brief If the does not filterchain exists, create it.
   * \param filterchain The name of the filterchain to create.
   * \return Either if the filterchain was added or not.
   */
  bool CreateFilterchain(const std::string &filterchain);

  /**
   * \brief If the filterchain exists, delete it.
   * \param filterchain The name of the filterchain to delete.
   * \return Either if the filterchain was delete or not.
   */
  bool DeleteFilterchain(const std::string &filterchain);

  /**
   * \brief Check if a filterchain exist or not.
   *
   * This will check on the list of the available filterchain provided by
   * GetAvailableFilterchain if the file exists or not.
   *
   * \param filterchain The name of the filterchain to check.
   * \return Either if the file exist or not
   */
  bool FilterchainExists(const std::string &filterchain);

  /**
   * \brief With the constants defining the config directory path and the
   * extension, return the true path of a filterchain.
   */
  std::string GetFilterchainPath(const std::string &filterchain) const;

  /**
   * \brief Display all the available filterchains on the standard output.
   */
  void ListAvailableFilterchains();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * \brief Copies a filterchain which is not used by a running execution.
   *
   * Manages the ROS service vision_server_copy_filterchain.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name_to_copy	The name of the filterchain to copy.
   *  * filter_chain_new_name	The name of the new filterchain (the copy).
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackCopyFc(vision_server_copy_filterchain::Request &rqst,
                      vision_server_copy_filterchain::Response &rep);

  /**
   * \brief Gets the parameters for a filter.
   *
   * Manages the ROS service vision_server_get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * filter_name Name of the filter contained in the filterchain.
   *  * filter_chain_name Name of the filterchain used by the execution.
   *  * execution_name Name of the execution which is running.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFilterParam(
      vision_server_get_filterchain_filter_param::Request &rqst,
      vision_server_get_filterchain_filter_param::Response &rep);

  /**
   * TODO Thibaut Mattio: Check this method, this is exacly the same
   * as the above one, is it really usefull ???
   *
   * \brief Gets the parameters for a filter.
   *
   * Manages the ROS service vision_server_get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * filter_name Name of the filter contained in the filterchain.
   *  * filter_chain_name Name of the filterchain used by the execution.
   *  * execution_name Name of the execution which is running.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFilterAllParam(
      vision_server_get_filterchain_filter_all_param::Request &rqst,
      vision_server_get_filterchain_filter_all_param::Response &rep);

  /**
   * \brief Set the value of a parameter of a filter contained in a filterchain
   * used by a running execution.
   *
   * Manages the ROS service vision_server_set_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name Name of the filterchain which contain the filter.
   *  * filter_name Name of the filter which contain the parameter.
   *  *	parameter_name Name of the parameter which the value has to be set.
   *  *	parameter_value The value of the parameter.
   *  *	execution_name Name of the running execution using the filterchain.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSetFilterParam(
      vision_server_set_filterchain_filter_param::Request &rqst,
      vision_server_set_filterchain_filter_param::Response &rep);

  /**
   * \brief Gets the filters contained in a filterchain.
   *
   * Manages the ROS service vision_server_get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  *	filter_chain_name Name of the filterchain.
   *  *	execution_name Name of the execution. Let empty to get the filters list
   *                   associated with a filterchain.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFilter(vision_server_get_filterchain_filter::Request &rqst,
                         vision_server_get_filterchain_filter::Response &rep);

  /**
   * \brief Adds/Deletes a filter in a filterchain used by a running execution.
   *
   * Manages the ROS service vision_server_manage_filterchain_filter.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name	Name of the filterchain.
   *  * commande The commande to proccess. 1 for ADD, 2 for DELETE.
   *  * filter_name Name of the filter.
   *  * execName Name of the execution.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackManageFilter(
      vision_server_manage_filterchain_filter::Request &rqst,
      vision_server_manage_filterchain_filter::Response &rep);

  /**
   * \brief Creates/Deletes a filterchain (the .fc fils in config directory).
   *
   * Manages the ROS service vision_server_manage_filterchain.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name	Name of the filterchain.
   *  * commande The commande to process. 1 for ADD, 2 for DELETE.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackManageFc(vision_server_manage_filterchain::Request &rqst,
                        vision_server_manage_filterchain::Response &rep);

  /**
   * \brief Saves a filterchain.
   *
   *
   * When a filterchain is used by a running execution, the filters and their
   * parameters values can be modified.
   * The VisionServer doesn't store the new values until the user click to the
   * save button.
   * This will call this method, which send a request to the VisionServer
   * in order to save the filterchain.
   * If the filterchain is not used by a running execution, this call will
   * fail.
   *
   * Manages the ROS service vision_server_save_filterchain.
   *
   * Here are the parameters of the service:
   *  * filter_chain_name Name of the filterchain to save.
   *  * execution_name Name of the execution which use the filterchain to
   *                   save.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSaveFc(vision_server_save_filterchain::Request &rqst,
                      vision_server_save_filterchain::Response &rep);

  /**
   * \brief Change the order of a filter in a filterchain.
   *
   * Manages the ROS service vision_server_set_filterchain_filter_order.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the execution which use the filterchain.
   *  * filter_chain_name	Name of the filterchain which contains the
   *                      filter to move.
   *  * filter_index Zero-based index of the filter.
   *  * commande 1=UP, 2=DOWN.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSetFcOrder(
      vision_server_set_filterchain_filter_order::Request &rqst,
      vision_server_set_filterchain_filter_order::Response &rep);

  /**
   * \brief Get the filterchain used by the running execution given as
   *parameter.
   *
   * Manages the ROS service vision_server_get_filterchain_from_execution.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the running execution.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetFcFromExec(
      vision_server_get_filterchain_from_execution::Request &rqst,
      vision_server_get_filterchain_from_execution::Response &rep);

  /**
   * \brief Get the media used by the running execution given as parameter.
   *
   * Manages the ROS service vision_server_get_media_from_execution.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the running execution.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackGetMediaFromExec(
      vision_server_get_media_from_execution::Request &rqst,
      vision_server_get_media_from_execution::Response &rep);

  /**
   * \brief Sets the observer to the filter given as parameter.
   *
   * As the processing of a filterchain is like a pipe, it is possible to
   * observe the render of a specific filter and all the filters before.
   * This method set this "cursor".
   * The filter has to be used by a filterchain used by a running execution.
   *
   * Manages the ROS service vision_server_get_filterchain_filter_param.
   *
   * Here are the parameters of the service:
   *  * execution_name Name of the running execution.
   *  * filter_chain_name Name of the filterchain used by the execution.
   *  * filter_name Name of the filter used as the "cursor" of the observer.
   *
   * \param	rqst The ROS object containing the Request of the service.
   *             It contains all the parameters sent to the callback
   * \param	rep The ROS object containnig the Response of the service.
   *            It set values such as success for the service return state.
   * \return True if the callback has succesfully processed the service call.
   */
  bool CallbackSetObserver(
      vision_server_set_filterchain_filter_observer::Request &rqst,
      vision_server_set_filterchain_filter_observer::Response &rep);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * CameraConfig _cam_config
   */
  const char *FILTERCHAIN_MANAGER_TAG;

  /**
   * List of current instances of filterchains
   */
  std::vector<Filterchain *> _runningFilterchains;

  /**
   * ROS logger
   */
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

#endif  // VISION_SERVER_FILTERCHAIN_MANAGER_H_