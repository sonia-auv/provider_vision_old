/**
 * \file	CAMConfig.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <string>
#include "media/cam_config.h"
#include "utils/pugixml.h"

namespace vision_server {

//==============================================================================
// C O N S T A N T S   S E C T I O N

static const std::string xml_cameraID_tag = "cameraID";
static const std::string xml_undistordMatrix_tag = "undistordMatrix";

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CAMConfig::CAMConfig(std::string file) {
  std::string cameraConfigPath = file + "camera_config.xml";
  pugi::xml_document doc;
  if (doc.load_file(cameraConfigPath.c_str(), pugi::parse_default)) {
    pugi::xml_node camera = doc.child("camera_list").child("camera");
    for (; camera; camera = camera.next_sibling()) {
      std::string cam_name = "";
      auto cam_guid = uint64_t{0};
      std::string cam_undistord_matrices_path = "";
      auto child = camera.first_child();
      // For each node in the camera description
      for (; child != nullptr; child = child.next_sibling()) {
        auto child_name = child.name();
        if (child_name == xml_cameraID_tag) {
          cam_name = child.first_attribute().value();
          std::stringstream ss;
          ss << std::hex << child.first_attribute().next_attribute().value();
          ss >> cam_guid;
        }
        if (child_name == xml_undistordMatrix_tag) {
          std::stringstream ss;
          ss << file << child.first_attribute().value();
          cam_undistord_matrices_path = ss.str();
        }
      }
      auto temp = CameraConfig{};
      temp.camID = CameraID(cam_name, cam_guid);
      // Really stupid. We construct it uninitialise, then we construct and copy
      // an initialize object... should be changed...
      temp.camID._camUndistordMatrices.InitMatrices(
          cam_undistord_matrices_path);
      _list.push_back(temp);
    }
  }
}

//------------------------------------------------------------------------------
//
CAMConfig::~CAMConfig() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
const std::vector<CameraConfig> CAMConfig::GetConfigList() { return _list; }

//------------------------------------------------------------------------------
//
CameraConfig *CAMConfig::GetConfig(uint64_t guid) {
  for (auto &config : _list) {
    if (config.camID.GetGUID() == guid) {
      return &config;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
CameraConfig *CAMConfig::GetConfig(std::string name) {
  for (auto &config : _list) {
    if (config.camID.GetName() == name) {
      return &config;
    }
  }
  return nullptr;
}

}  // namespace vision_server
