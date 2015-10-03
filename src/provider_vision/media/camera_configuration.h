/*
 * CameraConfiguration.h
 *
 *  Created on: Sep 10, 2015
 *      Author: jeremie
 */

#ifndef PROVIDER_VISION_camera_configuration_
#define PROVIDER_VISION_camera_configuration_

#include <map>
#include <memory>
#include "pugixml.hpp"

namespace vision_server {

  class CameraConfiguration {
  public:
    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    using Ptr = std::shared_ptr<CameraConfiguration>;

    CameraConfiguration(const std::string &name);
    virtual ~CameraConfiguration();

    void SetGUID(uint64_t guid);
    uint64_t GetGUID() const;

    void SetName(const std::string &name);
    const std::string &GetName() const;

    void SetUndistortionMatricePath(const std::string path);
    std::string GetUndistortionMatricePath() const;

    void AddConfiguration(const std::string &config_name,
                          const std::string &value);
    const std::map<std::string, std::string> &GetConfigurations() const;

    float GetFloat(const std::string &config_name) const;
    int GetInteger(const std::string &config_name) const;
    bool GetBoolean(const std::string &config_name) const;
    std::string GetString(const std::string &config_name) const;

  private:
    std::map<std::string, std::string> configuration_;
    uint64_t guid_;
    std::string name_;
    std::string undistortion_matrice_path_;
  };

  //=============================================================================
  //

  //-----------------------------------------------------------------------------
  //
  inline void CameraConfiguration::AddConfiguration(
      const std::string &config_name, const std::string &value) {
    configuration_.insert(
        std::pair<std::string, std::string>(config_name, value));
  }

  //-----------------------------------------------------------------------------
  //
  inline const std::map<std::string, std::string>
  &CameraConfiguration::GetConfigurations() const {
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
  inline void CameraConfiguration::SetName(const std::string &name){ name_ = name; };

  //-----------------------------------------------------------------------------
  //
  inline const std::string &CameraConfiguration::GetName() const{ return name_; }

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
}
#endif /* PROVIDER_VISION_SRC_MEDIA_CAMERA_CONFIGURATION_H_ */
