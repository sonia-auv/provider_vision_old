/**
 * \file	ros_callback_manager.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	23/05/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_ROS_CALLBACK_MANAGER_H_
#define VISION_SERVER_ROS_CALLBACK_MANAGER_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <ros/ros.h>

namespace vision_server {

//==============================================================================
// C L A S S E S

template <class SpecificManager>
class ROSCallbackManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  template <typename MReq, typename MRes>
  using CallBackPtr = bool (SpecificManager::*)(MReq &, MRes &);

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ROSCallbackManager();

  virtual ~ROSCallbackManager();

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  template <typename MReq, typename MRes>
  void RegisterService(std::string name, CallBackPtr<MReq, MRes> function,
                       SpecificManager &manager);

  //==========================================================================
  // P R O T E C T E D   M E M B E R S

  ros::NodeHandle _hdl;
  /**
   * List of ROS services offered by this class
   */
  std::map<std::string, ros::ServiceServer> _services;
};

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

// /!\ As this class is a template class, you can't simply put definition in
//     cpp files.
//     Here are the definitions of the class members.

//------------------------------------------------------------------------------
//
template <class SpecificManager>
ROSCallbackManager<SpecificManager>::ROSCallbackManager()
    : _hdl() {}

//------------------------------------------------------------------------------
//
template <class SpecificManager>
ROSCallbackManager<SpecificManager>::~ROSCallbackManager() {
  for (auto &service : _services) {
    service.second.shutdown();
  }
}

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
template <class SpecificManager>
template <typename MReq, typename MRes>
inline void ROSCallbackManager<SpecificManager>::RegisterService(
    std::string name, CallBackPtr<MReq, MRes> function,
    SpecificManager &manager) {
  auto result_advertise =
      _hdl.advertiseService(name.c_str(), function, &manager);
  auto pair =
      std::pair<std::string, ros::ServiceServer>(name, result_advertise);
  _services.insert(pair);
}

}  // namespace vision_server

#endif  // VISION_SERVER_ROS_CALLBACK_MANAGER_H_
