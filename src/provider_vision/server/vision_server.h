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

#include <lib_atlas/ros/service_server_manager.h>
#include <vision_server/vision_server_execute_cmd.h>
#include <vision_server/vision_server_get_information_list.h>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/proc/detection_task.h"
#include "provider_vision/server/media_manager.h"
#include "provider_vision/server/filterchain_manager.h"
#include "provider_vision/server/detection_task_manager.h"

namespace vision_server {

/**
 * Vision server is the main class of the system
 * It's job is to assemble and connect the pieces to create execution
 * It is the owner of the active DetectionTask and MediaStreamer
 * It does not hold the filterchains, as it is the responsability of the
 * filterchain manager.
 * It gets the filterchains from the filterchain manager.
 *
 * The visionServer is also the service point for listing service (media, exec,
 * filterchain and filter)
 * and the start/stop execution.
 */
class VisionServer : public atlas::ServiceServerManager<VisionServer> {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  explicit VisionServer(atlas::NodeHandlePtr node_handle);

  ~VisionServer();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Return a string of each execution separated by the current
   * COMPONENT_SEPARATOR given a list of the executions.
   */
  std::string BuildRosMessage(
      const std::vector<DetectionTask> &detection_tasks) const noexcept;

  std::string BuildRosMessage(
      const std::vector<Filterchain> &filterchains) const noexcept;

  std::string BuildRosMessage(const std::vector<BaseMedia> &medias) const
      noexcept;

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

  bool CallbackExecutionCMD(
      vision_server::vision_server_execute_cmd::Request &rqst,
      vision_server::vision_server_execute_cmd::Response &rep);

  bool CallbackInfoListCMD(
      vision_server::vision_server_get_information_list::Request &rqst,
      vision_server::vision_server_get_information_list::Response &rep);

  /**
   * Answer to the service get media params
   */
  bool CallbackGetCMD(vision_server_get_media_param::Request &rqst,
                      vision_server_get_media_param::Response &rep);

  /**
   * Answer to the service asking to set a parameter of a media.
   */
  bool CallbackSetCMD(vision_server_set_media_param::Request &rqst,
                      vision_server_set_media_param::Response &rep);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /**
   * We must keep a reference to the initial node_handle for creating the
   * different topics and services outside the constructor.
   * This is mainly for performance purpose as we could also recreate a
   * ros::NodeHandle on the different instance of the objects.
   */
  atlas::NodeHandlePtr node_handle_;

  MediaManager media_manager_;

  FilterchainManager filterchain_manager_;

  DetectionTaskManager detection_task_manager_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline std::string VisionServer::BuildRosMessage(
    const std::vector<DetectionTask> &detection_tasks) const noexcept {
  std::string msg = {""};
  for (const auto &detection_task : detection_tasks) {
    msg += detection_task.GetExecName() + ";";
  }
  return msg;
}

//------------------------------------------------------------------------------
//
inline std::string VisionServer::BuildRosMessage(
    const std::vector<Filterchain> &filterchains) const noexcept {
  std::string msg = {""};
  for (const auto &filterchain : filterchains) {
    msg += filterchain.GetName() + ";";
  }
  return msg;
}

//------------------------------------------------------------------------------
//
inline std::string VisionServer::BuildRosMessage(
    const std::vector<BaseMedia> &medias) const noexcept {
  std::string msg = {""};
  for (const auto &media : medias) {
    msg += media.GetName() + ";";
  }
  return msg;
}

}  // namespace vision_server

#endif  // VISION_SERVER_VISION_SERVER_H_
