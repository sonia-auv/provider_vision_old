/**
 * \file	filterchain.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/server/filterchain.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Filterchain::Filterchain(const std::string &name)
    : Serializable(kConfigPath + "filterchain/" + name + ".yaml"),
      name_(name),
      param_handler_(),
      observer_index_(0) {
  Deserialize();
  observer_index_ = filters_.size() - 1;
}

//------------------------------------------------------------------------------
//
Filterchain::Filterchain(const Filterchain &filterchain)
    : Serializable(kConfigPath + filterchain.name_ + "_copy" + kFilterchainExt),
      name_(filterchain.name_ + "_copy"),
      param_handler_(filterchain.param_handler_),
      observer_index_(filterchain.observer_index_) {}

//------------------------------------------------------------------------------
//
Filterchain::~Filterchain() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool Filterchain::Serialize() {
  YAML::Node node;
  node["name"] = GetName();
  int i{0};
  for (auto &filter : filters_) {
    YAML::Node filter_node;
    filter_node["name"] = filter->GetName();
    filter_node["id"] = i;

    auto parameters = filter->GetParameters();
    for (const auto &parameter : parameters) {
      YAML::Node param_node;
      param_node["name"] = parameter->GetName();
      param_node["type"] = parameter->GetType();
      param_node["value"] = parameter->GetStringValue();
      filter_node["parameters"].push_back(param_node);
    }

    node["filters"].push_back(filter_node);
    ++i;
  }

  auto filepath = kFilterchainPath + GetName() + ".yaml";
  std::ofstream fout(filepath);
  fout << node;
  return true;
}

//------------------------------------------------------------------------------
//
bool Filterchain::Deserialize() {
  YAML::Node node = YAML::LoadFile(filepath_);

  if (node["name"]) {
    SetName(node["name"].as<std::string>());
  }

  auto filters = node["filters"];
  assert(filters.Type() == YAML::NodeType::Sequence);

  for (std::size_t i = 0; i < filters.size(); i++) {
    auto filter_node = filters[i];
    AddFilter(filter_node["name"].as<std::string>());

    auto parameters = filter_node["parameters"];
    assert(parameters.Type() == YAML::NodeType::Sequence);

    for (std::size_t j = 0; j < parameters.size(); j++) {
      auto param_node = parameters[j];

      auto param_name = param_node["name"].as<std::string>();
      auto param_value = param_node["value"].as<std::string>();
      SetFilterParameterValue(j, param_name, param_value);
    }
  }

  return true;
}

//------------------------------------------------------------------------------
//
void Filterchain::ExecuteFilterChain(cv::Mat &image) {
  cv::Mat imageToProcess = image.clone();
  if (!imageToProcess.empty()) {
    param_handler_.setOriginalImage(imageToProcess);

    auto it = filters_.begin();
    try {
      size_t index = 0;
      for (; it != filters_.end(); ++it) {
        if (!imageToProcess.empty()) {
          (*it)->Execute(imageToProcess);
        }

        if (index == observer_index_) {
          imageToProcess.copyTo(image);
        }

        index++;
      }
    } catch (cv::Exception &e) {
      auto filterchainID = std::string{"[FILTERCHAIN " + name_ + "]"};
      ROS_ERROR_NAMED(filterchainID.c_str(), "Error in image processing: %s",
                      e.what());
    };
  }
}

//------------------------------------------------------------------------------
//
void Filterchain::RemoveFilter(const size_t &index) {
  if (index <= filters_.size()) {
    auto it = filters_.begin() + index;
    filters_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
void Filterchain::MoveFilterDown(const size_t &index) {
  if (index < (filters_.size() - 1)) {
    auto itFilter = filters_.begin();
    std::advance(itFilter, index);

    auto itFilterBellow = filters_.begin();
    std::advance(itFilterBellow, index + 1);

    std::swap(*itFilter, *itFilterBellow);
  } else {
    std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
    ROS_WARN_NAMED(filterchainID.c_str(), "Can't move this filter down");
  }
}

//------------------------------------------------------------------------------
//
void Filterchain::MoveFilterUp(const size_t &index) {
  if ((index > 0) && (index <= (filters_.size() - 1))) {
    auto itFilter = filters_.begin();
    std::advance(itFilter, index);

    auto itFilterAbove = filters_.begin();
    std::advance(itFilterAbove, index - 1);

    std::swap(*itFilter, *itFilterAbove);
  } else {
    std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
    ROS_WARN_NAMED(filterchainID.c_str(), "Can't move this filter down");
  }
}

//------------------------------------------------------------------------------
//
std::string Filterchain::GetFilterParameterValue(
    const size_t &index, const std::string &param_name) {
  return GetFilter(index)->GetParameterValue(param_name);
}

//------------------------------------------------------------------------------
//
void Filterchain::SetFilterParameterValue(const size_t &index,
                                          const std::string &param_name,
                                          const std::string &param_value) {
  GetFilter(index)->SetParameterValue(param_name, param_value);
}

//------------------------------------------------------------------------------
//
std::vector<provider_vision::ParameterInterface *>
Filterchain::GetFilterAllParameters(const size_t &index) {
  return GetFilter(index)->GetParameters();
}

//------------------------------------------------------------------------------
//
void Filterchain::AddFilter(const std::string &filter_name) {
  auto filter = provider_vision::Filter::Ptr(
      provider_vision::FilterFactory::createInstance(filter_name,
                                                     param_handler_));
  if (filter != nullptr) {
    filters_.push_back(filter);
  } else {
    throw std::invalid_argument("This filter does not exist in the library");
  }
}

}  // namespace provider_vision
