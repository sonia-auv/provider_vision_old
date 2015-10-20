/**
 * \file	CAMConfig.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	05/11/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/configuration_handler.h"

namespace vision_server {

//==============================================================================
// C O N S T A N T S   S E C T I O N

static const std::string XML_CAMERA_LIST_TAG = "CameraList";
static const std::string XML_CAMERA_TAG = "Camera";
static const std::string XML_CAMERA_NAME_ATTRIBUTE = "name";
static const std::string XML_CAMERA_GUID_ATTRIBUTE = "guid";
static const std::string XML_CAMERA_UNDISTORTION_MATRICE_NODE =
    "UndistortionMatrice";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ConfigurationHandler::ConfigurationHandler(const std::string &file)
    : file_(file) {}

//------------------------------------------------------------------------------
//
ConfigurationHandler::~ConfigurationHandler() {}

//==============================================================================
// M E T H O D   S E C T I O N
//------------------------------------------------------------------------------
//
std::vector<CameraConfiguration> ConfigurationHandler::ParseConfiguration() {
  // If the file has been change since constructor
  if (!atlas::FileExists(file_)) {
    throw std::ios_base::failure("File not found or inaccessible");
  }

  pugi::xml_document doc;
  std::vector<CameraConfiguration> configuration_list;

  if (!doc.load_file(file_.c_str(), pugi::parse_default)) {
    throw std::ios_base::failure("File not found or inaccessible");
  }

  pugi::xml_node camera_list = doc.first_child();
  // Ensure we get Camera list tag
  while (camera_list &&
         std::string(camera_list.name()).compare(XML_CAMERA_LIST_TAG) != 0) {
    camera_list = doc.next_sibling();
  }

  pugi::xml_node camera = camera_list.first_child();

  for (; camera; camera = camera.next_sibling()) {
    std::string name;
    CameraConfiguration camera_config(name);

    // ATTRIBUTES
    // Parse the attribute for the name and the GUID
    for (auto attr = camera.first_attribute(); attr;
         attr = attr.next_attribute()) {
      if (XML_CAMERA_NAME_ATTRIBUTE.compare(attr.name()) == 0) {
        name = attr.value();
        camera_config.SetName(name);
      }

      if (XML_CAMERA_GUID_ATTRIBUTE.compare(attr.name()) == 0) {
        std::stringstream ss;
        ss << std::hex << attr.value();
        uint64_t guid;
        ss >> guid;
        camera_config.SetGUID(guid);
      }
    }

    // NODES
    for (auto config_element = camera.first_child(); config_element;
         config_element = config_element.next_sibling()) {
      auto attrib = config_element.first_attribute();

      if (attrib.empty()) continue;

      if (XML_CAMERA_UNDISTORTION_MATRICE_NODE.compare(config_element.name()) ==
          0) {
        std::string pathUndistord = kProjectPath + attrib.value();
        camera_config.SetUndistortionMatricePath(pathUndistord);
      } else {
        camera_config.AddConfiguration(config_element.name(), attrib.value());
      }
    }

    configuration_list.push_back(camera_config);
  }

  return configuration_list;
}

//------------------------------------------------------------------------------
//
void ConfigurationHandler::SaveConfiguration(
    const std::vector<CameraConfiguration> &system_config) {
  std::string orignal_file = file_;
  // If the file has been change since constructor
  size_t pos = file_.find_last_of(".");

  if (pos != std::string::npos) {
    file_ = file_.substr(0, pos);
  }
  file_ += "_tmp.xml";

  pugi::xml_document doc;
  pugi::xml_node camera_list = doc.append_child();
  camera_list.set_name(XML_CAMERA_LIST_TAG.c_str());
  for (auto &config : system_config) {
    auto camera_node = camera_list.append_child();
    camera_node.set_name(XML_CAMERA_TAG.c_str());

    // ATTRIBUTE
    auto attribute =
        camera_node.append_attribute(XML_CAMERA_NAME_ATTRIBUTE.c_str());
    attribute.set_value(config.GetName().c_str());
    attribute = camera_node.append_attribute(XML_CAMERA_GUID_ATTRIBUTE.c_str());
    std::stringstream ss;
    ss << std::hex << config.GetGUID();
    attribute.set_value(ss.str().c_str());

    auto undistord_matrice_node = camera_node.append_child();
    undistord_matrice_node.set_name(
        XML_CAMERA_UNDISTORTION_MATRICE_NODE.c_str());
    undistord_matrice_node.set_value(
        config.GetUndistortionMatricePath().c_str());

    // NODE
    for (const auto &config_element : config.GetConfigurations()) {
      auto config_element_node = camera_node.append_child();
      config_element_node.set_name(config_element.first.c_str());
      auto config_element_attr = config_element_node.append_attribute("value");
      config_element_attr.set_value(config_element.second.c_str());
    }
  }

  doc.save_file(file_.c_str());

  std::rename(file_.c_str(), orignal_file.c_str());
}

}  // namespace vision_server
