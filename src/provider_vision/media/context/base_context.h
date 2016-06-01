/// \author	Pierluc Bédard <pierlucbed@gmail.com>
/// \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#ifndef PROVIDER_VISION_MEDIA_CONTEXT_BASE_CONTEXT_H_
#define PROVIDER_VISION_MEDIA_CONTEXT_BASE_CONTEXT_H_

#include <lib_atlas/pattern/runnable.h>
#include <boost/any.hpp>
#include <memory>
#include <mutex>
#include "provider_vision/media/camera/base_camera.h"
#include "provider_vision/media/camera/base_media.h"

namespace provider_vision {

/**
 * Base class for any media driver. It also provide a Camera interface
 * which enhance Media class' basic method with camera handling method.
 */
class BaseContext : public atlas::Runnable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BaseContext>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  BaseContext() = default;

  virtual ~BaseContext() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void CloseContext() = 0;

  virtual bool OpenMedia(const std::string &name) = 0;

  virtual bool CloseMedia(const std::string &name) = 0;

  virtual bool StartStreamingMedia(const std::string &name) = 0;

  virtual bool StopStreamingMedia(const std::string &name) = 0;

  virtual bool GetFeature(const BaseCamera::Feature &feat,
                          const std::string &name, boost::any &val) const = 0;

  virtual bool SetFeature(const BaseCamera::Feature &feat,
                          const std::string &name, boost::any &val) = 0;

  /**
   * Method to get all listed (connected) camera of the system
   */
  virtual std::vector<BaseMedia::Ptr> GetMediaList() const;

  /**
   * \return The camera if it is open.
   *         WILL NOT OPEN IT IF NOT.
   */
  virtual BaseMedia::Ptr GetMedia(const std::string &name) const;

  /**
   * Utility to know if this ID is associated to the media.
   */
  virtual bool ContainsMedia(const std::string &nameMedia) const;

  virtual bool WatchDogFunc() = 0;

 protected:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  virtual void EraseMedia(const std::string &name_media);

  std::vector<BaseMedia::Ptr> media_list_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline bool BaseContext::ContainsMedia(const std::string &nameMedia) const {
  if (GetMedia(nameMedia)) {
    return true;
  }
  return false;
}

//-----------------------------------------------------------------------------
//
inline std::vector<BaseMedia::Ptr> BaseContext::GetMediaList() const {
  return media_list_;
}

//-----------------------------------------------------------------------------
//
inline BaseMedia::Ptr BaseContext::GetMedia(const std::string &name) const {
  BaseMedia::Ptr media(nullptr);

  for (auto &elem : media_list_) {
    if (elem.get()->GetName().compare(name) == 0) {
      media = elem;
    }
  }
  return media;
}

//-----------------------------------------------------------------------------
//
inline void BaseContext::EraseMedia(const std::string &name_media) {
  for (auto iter = media_list_.begin(); iter != media_list_.end(); iter++) {
    if (iter->get()->GetName().compare(name_media) == 0) {
      media_list_.erase(iter);
      return;
    }
  }
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_CAM_DRIVER_H_
