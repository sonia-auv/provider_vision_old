//
// Created by parallels on 5/4/16.
//

#ifndef PROVIDER_VISION_GIGE_CONTEXT_H
#define PROVIDER_VISION_GIGE_CONTEXT_H

#include <inttypes.h>
#include <GenApi/GenApi.h>
#include <gevapi.h>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/camera/gige_camera.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/utils/config.h"

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

  void SetFeature(BaseCamera::Feature feat, const std::string &name,
                  float val) override;

  void GetFeature(BaseCamera::Feature feat, const std::string &name,
                  float &val) const override;

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
