/**
 * \file	FilterChain.h
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_FILTERCHAIN_H_
#define PROVIDER_VISION_FILTERCHAIN_H_

#include <opencv2/opencv.hpp>
#include <lib_vision/filter_factory.h>
#include <lib_vision/global_param_handler.h>
#include <lib_vision/filter.h>
#include "provider_vision/utils/serializable.h"

namespace vision_server {

class Filterchain : public Serializable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Filterchain>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Filterchain(const std::string &name, const std::string &execution);


  explicit Filterchain(const Filterchain &filterchain);

  ~Filterchain();

  //============================================================================
  // P U B L I C   M E T H O D S

  bool Serialize() override;

  bool Deserialize() override;

  /**
   * For all the filters in the list of filters. This will call the init method
   * in order to set the filter up.
   */
  void InitFilters();

  /**
   * For all the filters in the list of the filters, This will call the destroy
   * method in order to shut the filter down.
   */
  void CloseFilters();

  /**
   * Given a name of a filter, this will return the first filter with the same
   * name in the filterchain.
   * CAUTION: It is possible that there is several filters with the same name
   * inside the filterchain. In that case it would be more appropriate to return
   * a collection of filters.
   * TODO Thibaut Mattio: Change this method to return a vector instead of
   * the first item in the list.
   *
   * \param filter_name The name of the filter to get.
   * \return A pointer to the filter if it exists, nullptr if not.
   */
  vision_filter::Filter *const GetFilter(const std::string &filter_name) const;

  /**
   * Check if there is a filter with the same name than the given parameter.
   *
   * \param filter_name The name of the filter to check.
   * \return Either if a filter with the same name exists or not.
   */
  bool HasFilter(const std::string &filter_name) const;

  std::string ExecuteFilterChain(cv::Mat &image);

  void SetObserver(const std::string &filterName);

  void AddFilter(const std::string &filter_name);

  void RemoveFilter(const size_t index);

  void MoveFilterDown(const size_t filterIndex);

  void MoveFilterUp(const size_t filterIndex);

  // Getter for the filter list in string (for the client)
  std::string GetFilterList();

  // Return all the parameter of a filter.
  std::string GetFilterAllParam(const std::string &filter_name);

  // Getter/setter for individual param (for the client)
  std::string GetFilterParam(const std::string &filter_name,
                             const std::string &param_name);

  void SetFilterParam(const std::string &filter_name,
                      const std::string &param_name,
                      const std::string &param_value);

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  void SetExecutionName(const std::string &executionName);

  // Communication from the UI goes on the form
  size_t GetFilterIndexFromUIName(const std::string &name) const;

  std::string GetName() const;

  void SetName(const std::string &name);

  /**
   * GlobalParams
   */
  vision_filter::GlobalParamHandler *getParamHandler();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string name_;

  vision_filter::GlobalParamHandler _global_params;

  std::vector<vision_filter::Filter *> filters_;

  int _observer_index;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void Filterchain::InitFilters() {
  for (auto &filter : filters_) {
    if (filter != nullptr) {
      filter->init();
    }
  }
}

//------------------------------------------------------------------------------
//
inline void Filterchain::CloseFilters() {
  for (auto &filter : filters_) {
    if (filter != nullptr) {
      filter->destroy();
    }
  }
}

//------------------------------------------------------------------------------
//
inline vision_filter::Filter *const Filterchain::GetFilter(
    const std::string &filter_name) const {
  for (const auto &filter : filters_) {
    if (filter->getName() == filter_name) {
      return filter;
    }
  }
  return nullptr;
}

//------------------------------------------------------------------------------
//
inline bool Filterchain::HasFilter(const std::string &filter_name) const {
  for (const auto &filter : filters_) {
    if (filter->getName() == filter_name) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void Filterchain::SetObserver(const std::string &filterName) {
  _observer_index = GetFilterIndexFromUIName(filterName);
}

//------------------------------------------------------------------------------
//
inline vision_filter::GlobalParamHandler *Filterchain::getParamHandler() {
  return &_global_params;
}

//------------------------------------------------------------------------------
//
inline std::string Filterchain::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
inline void Filterchain::SetName(const std::string &name) { name_ = name; }

//------------------------------------------------------------------------------
//
inline size_t Filterchain::GetFilterIndexFromUIName(
    const std::string &name) const {
  // So a filer name goes like this: #_filterName,
  // where the # is the position in the list, 0 based.
  // So we need to strip the number and get the coressponding filter.
  size_t pos = name.find("_");
  // Did not find the filter index.
  if (pos == std::string::npos) {
    return std::string::npos;
  }
  std::string position = name.substr(pos + 1, name.size() - 1);
  return size_t(atoi(position.c_str()));
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_FILTERCHAIN_H_
