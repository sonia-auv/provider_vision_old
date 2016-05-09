#include "provider_vision/media/context/dc1394_context.h"
#include "gige_context.h"
#include <ros/ros.h>
#include <string>
#include <vector>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
GigeContext::GigeContext(const std::vector<CameraConfiguration> &configurations) noexcept : BaseContext(), DRIVER_TAG("[GigE Driver]"), driver_(nullptr){
  InitContext(configurations);
}

//------------------------------------------------------------------------------
//

GigeContext::~GigeContext() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void GigeContext::InitContext(const std::vector<CameraConfiguration> &configurations) {

}

//------------------------------------------------------------------------------
//
void GigeContext::CloseContext() {

}

//------------------------------------------------------------------------------
//
void GigeContext::OpenMedia(const std::string &name) {

}

//------------------------------------------------------------------------------
//
void GigeContext::CloseMedia(const std::string &name) {

}

//------------------------------------------------------------------------------
//
void GigeContext::StartStreamingMedia(const std::string &name) {

}

//------------------------------------------------------------------------------
//
void GigeContext::StopStreamingMedia(const std::string &name) {

}

//------------------------------------------------------------------------------
//
void GigeContext::Run() {

}

//------------------------------------------------------------------------------
//
bool GigeContext::WatchDogFunc() {

}

//------------------------------------------------------------------------------
//
void GigeContext::SetFeature(BaseCamera::Feature feat, const std::string &name, float val) {

}

//------------------------------------------------------------------------------
//
void GigeContext::GetFeature(BaseCamera::Feature feat, const std::string &name, float &val) const {

}

}  // namespace provider_vision