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
    : BaseContext(), webcam_(std::make_shared<WebcamCamera>()) {}

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
void WebcamContext::SetFeature(const BaseCamera::Feature &feat,
                               const std::string &name, boost::any &val) {}

//------------------------------------------------------------------------------
//
void WebcamContext::GetFeature(const BaseCamera::Feature &feat,
                               const std::string &name, boost::any &val) const {}

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
