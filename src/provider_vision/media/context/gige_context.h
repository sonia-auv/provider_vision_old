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

#ifndef PROVIDER_VISION_GIGE_CONTEXT_H
#define PROVIDER_VISION_GIGE_CONTEXT_H

#include <GenApi/GenApi.h>
#include <gevapi.h>
#include <inttypes.h>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/camera/gige_camera.h"
#include "provider_vision/media/context/base_context.h"

namespace provider_vision {

class GigeContext : public BaseContext {
 public:
  const std::string DRIVER_TAG;

  const double TIME_FOR_BUS_ERROR = 3;
  static const int MAX_CAMERAS = 4;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<GigeContext>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit GigeContext(
      const std::vector<CameraConfiguration> &configurations) noexcept;

  virtual ~GigeContext() noexcept;

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

  GigeCamera::Ptr GetGigeCamera(const std::string &name) const;

  GigeCamera::Ptr GetGigeCamera(BaseMedia::Ptr media) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  GEV_CAMERA_INFO driver_[MAX_CAMERAS];
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
inline bool GigeContext::ContainsMedia(const std::string &nameMedia) const {
  for (const auto &cam : media_list_) {
    if (nameMedia.compare(cam->GetName()) == 0) {
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
//
inline GigeCamera::Ptr GigeContext::GetGigeCamera(
    const std::string &name) const {
  return GetGigeCamera(GetMedia(name));
}

//-----------------------------------------------------------------------------
//
inline GigeCamera::Ptr GigeContext::GetGigeCamera(BaseMedia::Ptr media) const {
  GigeCamera::Ptr tmp = std::dynamic_pointer_cast<GigeCamera>(media);

  // Should not happen since if we get here, we are probably in a for
  // loop that iters through media_list_
  // OR we received a name which returned true at ContainsMedia call
  // since it is the first step for calling camera function on a context
  if (!tmp) {
    throw std::invalid_argument("Media is not a GigE camera");
  }
  return tmp;
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_GIGE_CONTEXT_H
