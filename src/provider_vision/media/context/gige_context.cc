#include "provider_vision/media/context/dc1394_context.h"
#include "gige_context.h"
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <gevapi.h>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
GigeContext::GigeContext(
    const std::vector<CameraConfiguration> &configurations) noexcept
    : BaseContext(),
      DRIVER_TAG("[GigE Driver]") {
  InitContext(configurations);
}

//------------------------------------------------------------------------------
//

GigeContext::~GigeContext() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void GigeContext::InitContext(
    const std::vector<CameraConfiguration> &configurations) {
  ROS_INFO_NAMED(DRIVER_TAG, "Initializing GigE driver");
  UINT16 status;
  int numCamera = 0;

  media_list_.clear();

  status = GevGetCameraList(driver_, MAX_CAMERAS, &numCamera);

  if (status != GEVLIB_OK) {
    ROS_ERROR_NAMED(DRIVER_TAG, "Could not enumerate GigE cameras.");
    return;
  }

  if (numCamera == 0) {
    ROS_WARN_NAMED(DRIVER_TAG, "No GigE camera found.");
    return;
  }
  ROS_INFO_NAMED(DRIVER_TAG, "%d GigE camera found", numCamera);
  GEV_CAMERA_HANDLE camera = NULL;
  for (uint i = 0; i < numCamera; i++) {
    std::string name = driver_[i].username;
    for (auto const &cam_config : configurations) {
      if (cam_config.name_ == name) {
        if (&camera == nullptr) {
          throw std::runtime_error("Error creating the GigE camera");
        }

        std::shared_ptr<GigeCamera> cam(new GigeCamera(&camera, cam_config));

        cam->Open();
        // cam->SetCameraParams();
        media_list_.push_back(cam);
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void GigeContext::CloseContext() {
  for (auto &media : media_list_) {
    GigeCamera::Ptr cam = GetGigeCamera(media);

    cam->StopStreaming();

    cam->Close();
  }

  media_list_.clear();

  GevApiUninitialize();
}

//------------------------------------------------------------------------------
//
void GigeContext::OpenMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  cam->Open();
}

//------------------------------------------------------------------------------
//
void GigeContext::CloseMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  cam->Close();
}

//------------------------------------------------------------------------------
//
void GigeContext::StartStreamingMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  cam->StartStreaming();
}

//------------------------------------------------------------------------------
//
void GigeContext::StopStreamingMedia(const std::string &name) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  cam->StopStreaming();
}

//------------------------------------------------------------------------------
//
void GigeContext::Run() {
  while (!MustStop()) {
    atlas::SecTimer::Sleep(5);
    if (!WatchDogFunc()) {
      ROS_WARN_NAMED(DRIVER_TAG, "Watchdog returned with error");
      atlas::SecTimer::Sleep(2);
    }
  }
}

//------------------------------------------------------------------------------
//
bool GigeContext::WatchDogFunc() {
  bool fail = false;

  for (auto &active_camera : media_list_) {
    GigeCamera::Ptr cam = GetGigeCamera(active_camera);

    if (cam->GetAcquistionTimerValue() > TIME_FOR_BUS_ERROR) {
      ROS_FATAL_NAMED(DRIVER_TAG, "Camera is not feeding");
      fail = true;
    }
  }
  return fail;
}

//------------------------------------------------------------------------------
//
void GigeContext::SetFeature(BaseCamera::Feature feat, const std::string &name,
                             float val) {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  cam->SetFeature(feat, val);
}

//------------------------------------------------------------------------------
//
void GigeContext::GetFeature(BaseCamera::Feature feat, const std::string &name,
                             float &val) const {
  GigeCamera::Ptr cam = GetGigeCamera(name);
  val = cam->GetFeature(feat);
}

}  // namespace provider_vision