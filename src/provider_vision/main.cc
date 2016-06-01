/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include <lib_atlas/ros/service_server_manager.h>
#include <ros/ros.h>
#include "provider_vision/server/vision_server.h"
#include "../../cfg/cpp/provider_vision/Camera_Parameters_Config.h"
#include <dynamic_reconfigure/server.h>

void CallBackDynamReconf(provider_vision::Camera_Parameters_Config &config,
                         uint32_t level) {
  ROS_INFO("Recongifure Request: %d", config.gain_value);
}

//------------------------------------------------------------------------------
//
int main(int argc, char **argv) {
  ros::init(argc, argv, "provider_vision");
  ros::NodeHandle nh("~");

  provider_vision::VisionServer pv(nh);

  dynamic_reconfigure::Server<provider_vision::Camera_Parameters_Config> server;
  dynamic_reconfigure::Server<
      provider_vision::Camera_Parameters_Config>::CallbackType cb;

  cb = boost::bind(&CallBackDynamReconf, _1, _2);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
