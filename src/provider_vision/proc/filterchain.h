/**
 * \file	filterchain.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_PROC_FILTERCHAIN_H_
#define PROVIDER_VISION_PROC_FILTERCHAIN_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <queue>

#include "provider_vision/utils/pugixml.h"
#include <lib_vision/filter_factory.h>
#include <lib_vision/global_param_handler.h>
#include <lib_vision/filter.h>
#include "provider_vision/utils/serializable.h"
#include "provider_vision/utils/config.h"
#include "lib_vision/target.h"

namespace provider_vision {

class Filterchain : public Serializable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Filterchain>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Filterchain(const std::string &name);

  explicit Filterchain(const Filterchain &filterchain);

  ~Filterchain();

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Get the name of the filterchain.
   *
   * \return The name of the filterchain.
   */
  const std::string &GetName() const;

  /**
   * Set the name of the filterchain.
   *
   * \param name The new name of the filterchain.
   */
  void SetName(const std::string &name);

  bool Serialize() override;

  bool Deserialize() override;

  lib_vision::Filter::Ptr GetFilter(const size_t &index) const;

  std::vector<lib_vision::Filter::Ptr> GetFiltersWithName(
      const std::string &filter_name) const;

  std::vector<lib_vision::Filter::Ptr> GetAllFilters() const;

  /**
   * Check if there is a filter with the same name than the given parameter.
   *
   * \param filter_name The name of the filter to check.
   * \return Either if a filter with the same name exists or not.
   */
  bool ContainsFilter(const std::string &filter_name) const;

  void ExecuteFilterChain(cv::Mat &image);

  void SetObserver(const size_t &index);

  void AddFilter(const std::string &filter_name);

  void RemoveFilter(const size_t &index);

  void MoveFilterDown(const size_t &filterIndex);

  void MoveFilterUp(const size_t &filterIndex);

  std::string GetFilterParameterValue(const size_t &index,
                                      const std::string &param_name);

  void SetFilterParameterValue(const size_t &index,
                               const std::string &param_name,
                               const std::string &param_value);

  std::vector<lib_vision::ParameterInterface *> GetFilterAllParameters(
      const size_t &index);

  lib_vision::GlobalParamHandler::Ptr GetParameterHandler();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string name_;

  lib_vision::GlobalParamHandler param_handler_;

  std::vector<lib_vision::Filter::Ptr> filters_;

  size_t observer_index_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline lib_vision::Filter::Ptr Filterchain::GetFilter(
    const size_t &index) const {
  return filters_.at(index);
}

//------------------------------------------------------------------------------
//
inline std::vector<lib_vision::Filter::Ptr> Filterchain::GetFiltersWithName(
    const std::string &filter_name) const {
  std::vector<lib_vision::Filter::Ptr> filters;
  for (const auto &filter : filters_) {
    if (filter->GetName() == filter_name) {
      filters.push_back(filter);
    }
  }
  return filters;
}

//------------------------------------------------------------------------------
//
inline std::vector<lib_vision::Filter::Ptr> Filterchain::GetAllFilters() const {
  return filters_;
}

//------------------------------------------------------------------------------
//
inline bool Filterchain::ContainsFilter(const std::string &filter_name) const {
  for (const auto &filter : filters_) {
    if (filter->GetName() == filter_name) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void Filterchain::SetObserver(const size_t &index) {
  observer_index_ = index;
}

//------------------------------------------------------------------------------
//
inline lib_vision::GlobalParamHandler::Ptr Filterchain::GetParameterHandler() {
  return std::make_shared<lib_vision::GlobalParamHandler>(param_handler_);
}

//------------------------------------------------------------------------------
//
inline const std::string &Filterchain::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
inline void Filterchain::SetName(const std::string &name) { name_ = name; }

}  // namespace provider_vision

#endif  // PROVIDER_VISION_PROC_FILTERCHAIN_H_
