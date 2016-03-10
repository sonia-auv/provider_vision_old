/**
 * \file	webcam_context.cpp
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "provider_vision/media/context/webcam_context.h"
#include <string>
#include <vector>

namespace provider_vision {

const std::string WebcamContext::WEBCAM_NAME("Webcam");

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
WebcamContext::WebcamContext() noexcept
    : BaseContext(),
      webcam_(std::make_shared<WebcamCamera>()) {}

//------------------------------------------------------------------------------
//
WebcamContext::~WebcamContext() noexcept {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void WebcamContext::CloseContext() {}

//------------------------------------------------------------------------------
//
void WebcamContext::SetFeature(BaseCamera::Feature feat,
                               const std::string &name, float val) {}

//------------------------------------------------------------------------------
//
void WebcamContext::GetFeature(BaseCamera::Feature feat,
                               const std::string &name, float &val) const {}

//------------------------------------------------------------------------------
//
void WebcamContext::OpenMedia(const std::string &name) {
  if (WEBCAM_NAME.compare(name) == 0) {
    webcam_->Open();
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::CloseMedia(const std::string &name) {
  if (WEBCAM_NAME.compare(name) == 0) {
    webcam_->Close();
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::StartStreamingMedia(const std::string &name) {
  if (WEBCAM_NAME.compare(name) == 0) {
    webcam_->StartStreaming();
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::StopStreamingMedia(const std::string &name) {
  if (WEBCAM_NAME.compare(name) == 0) {
    webcam_->Close();
  }
}

//------------------------------------------------------------------------------
//
std::vector<BaseMedia::Ptr> WebcamContext::GetMediaList() const {
  return std::vector<BaseMedia::Ptr>({webcam_});
}

//------------------------------------------------------------------------------
//
BaseMedia::Ptr WebcamContext::GetMedia(const std::string &name) const {
  if (name == WEBCAM_NAME) {
    return webcam_;
  } else {
    return nullptr;
  }
}

//------------------------------------------------------------------------------
//
void WebcamContext::Run() {}

//------------------------------------------------------------------------------
//
bool WebcamContext::WatchDogFunc() { return true; }

}  // namespace provider_vision
