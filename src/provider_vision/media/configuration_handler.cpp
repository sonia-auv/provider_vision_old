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

#include <provider_vision/media/configuration_handler.h>

namespace vision_server {

//==============================================================================
// C O N S T A N T S   S E C T I O N

static const std::string XML_CAMERA_LIST_TAG = "Camera_list";
static const std::string XML_CAMERA_TAG = "Camera";
static const std::string XML_CAMERA_NAME_ATTRIBUTE = "name";
static const std::string XML_CAMERA_GUID_ATTRIBUTE = "guid";
static const std::string XML_CAMERA_UNDISTORTION_MATRICE_NODE =
    "UndistortionMatrice";

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ConfigurationHandler::ConfigurationHandler(const std::string &file) {
  if (!atlas::FileExists(file)) {
    throw std::ios_base::failure("File not found or inaccessible");
  }
  file_ = file;
}

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

  pugi::xml_node camera =
      doc.child(XML_CAMERA_LIST_TAG.c_str()).child(XML_CAMERA_TAG.c_str());

  for (; camera; camera = camera.next_sibling()) {
    std::string name;
    CameraConfiguration camera_config(name);

    // ATTRIBUTES
    // Parse the attribute for the name and the GUID
    for (auto attr = camera.first_attribute(); attr;
         attr = attr.next_attribute()) {
      if (XML_CAMERA_NAME_ATTRIBUTE.compare(attr.name()) == 0) {
        name = attr.value();
      }

      if (XML_CAMERA_GUID_ATTRIBUTE.compare(attr.name()) == 0) {
        std::stringstream ss;
        ss << std::hex << attr.value();
        uint64_t guid;
        ss >> guid;
        camera_config.SetGUID(guid);
        camera_config.SetName(name);
      }
    }

    // NODES
    for (auto config_element = camera.first_child(); config_element;
         config_element = config_element.next_sibling()) {
      auto attrib = config_element.first_attribute();

      if (attrib.empty()) continue;

      if (XML_CAMERA_UNDISTORTION_MATRICE_NODE.compare(config_element.name()) ==
          0) {
        camera_config.SetUndistortionMatricePath(attrib.value());
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
    const std::map<std::string, CameraConfiguration> &system_config) {
  std::string orignal_file = file_;
  // If the file has been change since constructor
  size_t pos = file_.find_last_of(".");

  if (pos != std::string::npos) {
    file_ = file_.substr(0, pos);
  }
  file_ += "_tmp.xml";

  pugi::xml_document doc;

  if (!doc.load_file(file_.c_str(), pugi::parse_default)) {
    throw std::ios_base::failure("File not found or inaccessible");
  }

  doc.append_child(XML_CAMERA_LIST_TAG.c_str());
  auto camera_list = doc.first_child();

  for (auto &config : system_config) {
    auto camera_node = camera_list.append_child(XML_CAMERA_TAG.c_str());
    CameraConfiguration configuration = config.second;

    // ATTRIBUTE
    auto attribute =
        camera_node.append_attribute(XML_CAMERA_NAME_ATTRIBUTE.c_str());
    attribute.set_value(config.first.c_str());
    attribute = camera_node.append_attribute(XML_CAMERA_GUID_ATTRIBUTE.c_str());
    unsigned long long int guid =
        static_cast<unsigned long long int>(configuration.GetGUID());
    attribute.set_value(guid);

    auto undistord_matrice_node =
        camera_node.append_child(XML_CAMERA_UNDISTORTION_MATRICE_NODE.c_str());
    undistord_matrice_node.set_value(
        configuration.GetUndistortionMatricePath().c_str());

    // NODE
    for (const auto &config_element : configuration.GetConfigurations()) {
      auto config_element_node =
          camera_node.append_child(config_element.first.c_str());
      config_element_node.set_value(config_element.second.c_str());
    }
  }

  doc.save_file(file_.c_str());

  if (!std::remove(orignal_file.c_str())) {
    throw "Error deleting file";
  }

  if (!std::rename(file_.c_str(), orignal_file.c_str())) {
    throw "Error renaming file";
  }
}

}  // namespace vision_server
