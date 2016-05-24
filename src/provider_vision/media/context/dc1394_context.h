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

#ifndef PROVIDER_VISION_MEDIA_CONTEXT_DC1394_CONTEXT_H_
#define PROVIDER_VISION_MEDIA_CONTEXT_DC1394_CONTEXT_H_

#include <dc1394/dc1394.h>
#include <inttypes.h>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/camera/dc1394_camera.h"
#include "provider_vision/media/context/base_context.h"

namespace provider_vision {

/**
 * DC1394Context is a driver to access all cameras that can
 * be used by the library DC1394. It does firewire and some
 * USB camera (if follow IIDC 1.31)
 * It manage the bus, the active camera.
 * It is the one creating and destroying cameras.
 */
class DC1394Context : public BaseContext {
 public:
  const std::string DRIVER_TAG;

  const double TIME_FOR_BUS_ERROR = 3;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DC1394Context>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit DC1394Context(
      const std::vector<CameraConfiguration> &configurations) noexcept;

  virtual ~DC1394Context() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void CloseContext() override;

  void OpenMedia(const std::string &name) override;

  void CloseMedia(const std::string &name) override;

  void StartStreamingMedia(const std::string &name) override;

  void StopStreamingMedia(const std::string &name) override;

  virtual void GetFeature(const BaseCamera::Feature &feat,
                          const std::string &name,
                          boost::any &val) const override;

  virtual void SetFeature(const BaseCamera::Feature &feat,
                          const std::string &name, boost::any &val) override;

  bool ContainsMedia(const std::string &nameMedia) const override;

  void Run() override;

  bool WatchDogFunc() override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void InitContext(const std::vector<CameraConfiguration> &configurations);

  DC1394Camera::Ptr GetDC1394Camera(const std::string &name) const;

  DC1394Camera::Ptr GetDC1394Camera(BaseMedia::Ptr media) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  dc1394_t *driver_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline bool DC1394Context::ContainsMedia(const std::string &nameMedia) const {
  for (const auto &cam : media_list_) {
    if (nameMedia.compare(cam->GetName()) == 0) {
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
//
inline DC1394Camera::Ptr DC1394Context::GetDC1394Camera(
    const std::string &name) const {
  return GetDC1394Camera(GetMedia(name));
}

//-----------------------------------------------------------------------------
//
inline DC1394Camera::Ptr DC1394Context::GetDC1394Camera(
    BaseMedia::Ptr media) const {
  DC1394Camera::Ptr tmp = std::dynamic_pointer_cast<DC1394Camera>(media);

  // Should not happen since if we get here, we are probably in a for
  // loop that iters through media_list_
  // OR we received a name which returned true at ContainsMedia call
  // since it is the first step for calling camera function on a context
  if (!tmp) {
    throw std::invalid_argument("Media is not a DC1394 camera");
  }
  return tmp;
}

}  // namespace provider_vision
#endif  // PROVIDER_VISION_CAM_DRIVER_DC1394_H_
