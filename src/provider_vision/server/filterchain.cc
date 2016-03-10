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

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Filterchain::Filterchain(const std::string &name)
    : Serializable(kConfigPath + "filterchain/" + name + kFilterchainExt),
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
  pugi::xml_document doc;
  auto node = doc.append_child("Filterchain");
  auto i = 0;
  for (auto &filter : filters_) {
    auto node_name = std::string{"Filter"};
    auto filter_node = node.append_child(node_name.c_str());
    auto filter_attr = filter_node.append_attribute("name");
    filter_attr.set_value(filter->GetName().c_str());
    for (const auto parameter : filter->GetParameters()) {
      auto parameter_node =
          filter_node.append_child(parameter->GetName().c_str());
      auto param_attr = parameter_node.append_attribute("value");
      param_attr.set_value(parameter->GetStringValue().c_str());
    }
    ++i;
  }
  auto save_path = kFilterchainPath + name_ + kFilterchainExt;
  doc.save_file(save_path.c_str());
  return true;
}

//------------------------------------------------------------------------------
//
bool Filterchain::Deserialize() {
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(filepath_.c_str());
  if (!result) {
    throw std::runtime_error("Error parsing file");
  }
  pugi::xml_node node = doc.child("Filterchain");

  auto filter = node.first_child();
  for (int i = 0; filter; filter = filter.next_sibling(), ++i) {
    auto attr = filter.first_attribute();
    for (; attr; attr = attr.next_attribute()) {
      if (std::string{attr.name()} == "value" ||
          std::string{attr.name()} == "name") {
        AddFilter(attr.value());
        auto parameter = filter.first_child();
        for (; parameter; parameter = parameter.next_sibling()) {
          auto attr_param = parameter.first_attribute();
          for (; attr_param; attr_param = attr_param.next_attribute()) {
            if (std::string{attr_param.name()} == "value") {
              SetFilterParameterValue(i, parameter.name(), attr_param.value());
            }
          }
        }
      }
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
