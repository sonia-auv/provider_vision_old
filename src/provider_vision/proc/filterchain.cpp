/**
 * \file	FilterChain.cpp
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "provider_vision/utils/pugixml.h"
#include "provider_vision/proc/filterchain.h"
#include "provider_vision/config.h"
#include <ros/ros.h>

namespace vision_server {

  //==============================================================================
  // C / D T O R S   S E C T I O N

  //------------------------------------------------------------------------------
  //
  Filterchain::Filterchain(const std::string &name, const std::string &execution)
  : Serializable(kConfigPath + name + kFilterchainExt),
    name_(name),
    _global_params(),
    _observer_index(0)
  {
    Deserialize();
    _observer_index = static_cast<int>(filters_.size() - 1);
  }

  //------------------------------------------------------------------------------
  //
  Filterchain::Filterchain(const Filterchain &filterchain)
  : Serializable(kConfigPath + filterchain.name_ + "_copy" + kFilterchainExt),
    name_(filterchain.name_ + "_copy"),
    _global_params(filterchain._global_params),
    _observer_index(filterchain._observer_index){}

  //==============================================================================
  // M E T H O D   S E C T I O N

  //------------------------------------------------------------------------------
  //
  bool Filterchain::Serialize() {
    pugi::xml_document doc;
    auto node = doc.append_child("Filterchain");
    auto i = 0;
    for (const auto filter : filters_) {
      auto node_name = std::string{"Filter"} + "_" + std::to_string(i);
      auto filter_node = node.append_child(node_name.c_str());
      auto filter_attr = filter_node.append_attribute("value");
      filter_attr.set_value(filter->getName().c_str());
      for (const auto parameter : filter->GetParameters()) {
        auto parameter_node =
            filter_node.append_child(parameter->getName().c_str());
        auto param_attr = parameter_node.append_attribute("value");
        param_attr.set_value(parameter->GetStringValue().c_str());
      }
      ++i;
    }
    auto save_path = kConfigPath + name_ + kFilterchainExt;
    doc.save_file(save_path.c_str());
    return true;
  }

  //------------------------------------------------------------------------------
  //
  bool Filterchain::Deserialize() {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filepath_.c_str());
    if( !result )
    {
      throw std::runtime_error("Error parsing file");
    }
    pugi::xml_node node = doc.child("Filterchain");

    auto filter = node.first_child();
    for (; filter; filter = filter.next_sibling()) {
      auto attr = filter.first_attribute();
      for (; attr; attr = attr.next_attribute()) {
        if (std::string{attr.name()} == "value") {
          // Ok, this is a filter and we can now create it and set the parameters
          // value from the child node of this one.
          AddFilter(attr.value());
          auto parameter = filter.first_child();
          for (; parameter; parameter = parameter.next_sibling()) {
            auto attr_param = parameter.first_attribute();
            for (; attr_param; attr_param = attr_param.next_attribute()) {
              if (std::string{attr_param.name()} == "value") {
                SetFilterParam(filter.name(), parameter.name(),
                               attr_param.value());
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
  std::string Filterchain::ExecuteFilterChain(cv::Mat &image) {
    std::string returnString;
    cv::Mat imageToProcess = image.clone();
    if (!imageToProcess.empty()) {
      _global_params.setOriginalImage(imageToProcess);

      auto it = filters_.begin();
      try {
        int filterIndex = 0;
        for (; it != filters_.end(); ++it) {
          if (!imageToProcess.empty()) {
            (*it)->execute(imageToProcess);
          }

          if (filterIndex == _observer_index) {
            imageToProcess.copyTo(image);
          }

          filterIndex++;
        }
      } catch (cv::Exception &e) {
        auto filterchainID = std::string{"[FILTERCHAIN " + name_ + "]"};
        ROS_ERROR_NAMED(filterchainID.c_str(), "Error in image processing: %s",
                        e.what());
      };
      returnString = _global_params.getNotifyString();
      _global_params.clearNotifyString();
    }
    return returnString;
  }

  //------------------------------------------------------------------------------
  //
  void Filterchain::RemoveFilter(const size_t index) {
    if (index <= filters_.size() && index >= 0) {
      auto it = filters_.begin() + index;
      filters_.erase(it);
    }
  }

  //------------------------------------------------------------------------------
  //
  void Filterchain::MoveFilterDown(const size_t filterIndex) {
    if ((filterIndex < (filters_.size() - 1)) && (filterIndex >= 0)) {
      auto itFilter = filters_.begin();
      std::advance(itFilter, filterIndex);

      auto itFilterBellow = filters_.begin();
      std::advance(itFilterBellow, filterIndex + 1);

      std::swap(*itFilter, *itFilterBellow);
    } else {
      std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
      ROS_WARN_NAMED(filterchainID.c_str(), "Can't move this filter down");
    }
  }

  //------------------------------------------------------------------------------
  //
  void Filterchain::MoveFilterUp(const size_t filterIndex) {
    if ((filterIndex > 0) && (filterIndex <= (filters_.size() - 1))) {
      auto itFilter = filters_.begin();
      std::advance(itFilter, filterIndex);

      auto itFilterAbove = filters_.begin();
      std::advance(itFilterAbove, filterIndex - 1);

      std::swap(*itFilter, *itFilterAbove);
    } else {
      std::string filterchainID = {"[FILTERCHAIN " + name_ + "]"};
      ROS_WARN_NAMED(filterchainID.c_str(), "Can't move this filter down");
    }
  }

  //------------------------------------------------------------------------------
  //
  std::string Filterchain::GetFilterList() {
    std::stringstream ss;
    int filter_count = 0;
    for (auto &elem : filters_) {
      // should never happen...
      if ((elem) == nullptr) {
        continue;
      }
      // Append the filter name to its index in the list etc.
      // This enables to identify the good filter with the good name if two
      // same
      // filter exist in the chain.
      ss << (elem)->getName() << "_" << filter_count << ";";
      filter_count++;
    }
    return ss.str();
  }

  //------------------------------------------------------------------------------
  //
  std::string Filterchain::GetFilterParam(const std::string &filter_name,
                                          const std::string &param_name) {
    size_t filter_index = GetFilterIndexFromUIName(filter_name);
    // Bad index...
    if (filter_index >= filters_.size() || filter_index == std::string::npos)
      return "";

    auto iter = filters_.begin();
    std::advance(iter, filter_index);
    std::string toReturn("");
    if (*iter != nullptr) {
      toReturn = (*iter)->getParamValue(param_name);
    }
    return toReturn;
  }

  //------------------------------------------------------------------------------
  //
  void Filterchain::SetFilterParam(const std::string &filter_name,
                                   const std::string &param_name,
                                   const std::string &param_value) {
    size_t filter_index = GetFilterIndexFromUIName(filter_name);
    // Bad index...
    if (filter_index >= filters_.size() || filter_index == std::string::npos) return;
    auto iter = filters_.begin();
    std::advance(iter, filter_index);
    // Should never happen...
    if ((*iter) != nullptr) {
      (*iter)->setParamValue(param_name, param_value);
    }
  }

  //------------------------------------------------------------------------------
  //
  std::string Filterchain::GetFilterAllParam(const std::string &filter_name) {
    size_t filter_index = GetFilterIndexFromUIName(filter_name);
    // Bad index...
    if (filter_index >= filters_.size() || filter_index == std::string::npos) {
      return "";
    }

    auto iter = filters_.begin();
    std::advance(iter, filter_index);
    std::string toReturn("");
    if (*iter != nullptr) {
      toReturn = (*iter)->getParamList();
    }
    return toReturn;
  }

  //------------------------------------------------------------------------------
  //
  void Filterchain::AddFilter(const std::string &filter_name) {
    auto filter =
        vision_filter::FilterFactory::createInstance(filter_name, _global_params);
    filters_.push_back(filter);
  }

}  // namespace vision_server
