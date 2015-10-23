/**
 * \file	camera_configuration.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/09/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef PROVIDER_VISION_MEDIA_CAMERA_CONFIGURATION_H_
#define PROVIDER_VISION_MEDIA_CAMERA_CONFIGURATION_H_

#include <map>
#include <memory>

namespace vision_server {

class CameraConfiguration {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CameraConfiguration>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit CameraConfiguration(const std::string &name);

  virtual ~CameraConfiguration();

  //==========================================================================
  // P U B L I C   M E T H O D S

  uint64_t GetGUID() const;

  const std::string &GetName() const;

  std::string GetUndistortionMatricePath() const;

  void SetGUID(uint64_t guid);

  void SetName(const std::string &name);

  void SetUndistortionMatricePath(const std::string path);

  float GetFloat(const std::string &config_name) const;

  int GetInteger(const std::string &config_name) const;

  bool GetBoolean(const std::string &config_name) const;

  std::string GetString(const std::string &config_name) const;

  void AddConfiguration(const std::string &config_name,
                        const std::string &value);

  const std::string &GetParam(const std::string &config_name) const;

  const std::map<std::string, std::string> &GetConfigurations() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::map<std::string, std::string> configuration_;

  uint64_t guid_;

  std::string name_;

  std::string undistortion_matrice_path_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline void CameraConfiguration::AddConfiguration(
    const std::string &config_name, const std::string &value) {
  configuration_.insert(
      std::pair<std::string, std::string>(config_name, value));
}

//-----------------------------------------------------------------------------
//
inline const std::map<std::string, std::string> &
CameraConfiguration::GetConfigurations() const {
  return configuration_;
}

//-----------------------------------------------------------------------------
//
inline void CameraConfiguration::SetGUID(uint64_t guid) { guid_ = guid; }

//-----------------------------------------------------------------------------
//
inline uint64_t CameraConfiguration::GetGUID() const { return guid_; }

//-----------------------------------------------------------------------------
//
inline void CameraConfiguration::SetName(const std::string &name) {
  name_ = name;
}

//-----------------------------------------------------------------------------
//
inline const std::string &CameraConfiguration::GetName() const { return name_; }

//-----------------------------------------------------------------------------
//
inline void CameraConfiguration::SetUndistortionMatricePath(
    const std::string path) {
  undistortion_matrice_path_ = path;
}

//-----------------------------------------------------------------------------
//
inline std::string CameraConfiguration::GetUndistortionMatricePath() const {
  return undistortion_matrice_path_;
}

//-----------------------------------------------------------------------------
//
inline const std::string &CameraConfiguration::GetParam(
    const std::string &config_name) const {
  auto config = configuration_.find(config_name);
  if (config == configuration_.end()) {
    throw std::invalid_argument("Param not found");
  }
  return config->second;
}

//-----------------------------------------------------------------------------
//
inline float CameraConfiguration::GetFloat(
    const std::string &config_name) const {
  return atof(GetParam(config_name).c_str());
}

//-----------------------------------------------------------------------------
//
inline int CameraConfiguration::GetInteger(
    const std::string &config_name) const {
  return atoi(GetParam(config_name).c_str());
}

//-----------------------------------------------------------------------------
//
inline bool CameraConfiguration::GetBoolean(
    const std::string &config_name) const {
  bool ret_val = false;
  if (GetParam(config_name).compare("True") == 0) ret_val = true;
  return ret_val;
}

//-----------------------------------------------------------------------------
//
inline std::string CameraConfiguration::GetString(
    const std::string &config_name) const {
  return GetParam(config_name);
}
}
#endif /* PROVIDER_VISION_SRC_MEDIA_CAMERA_CONFIGURATION_H_ */
