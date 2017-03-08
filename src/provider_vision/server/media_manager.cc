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

#include "provider_vision/server/media_manager.h"
#include "provider_vision/media/context/dc1394_context.h"
#include "provider_vision/media/context/gige_context.h"
#include "provider_vision/media/context/file_context.h"
#include "provider_vision/media/context/webcam_context.h"

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaManager::MediaManager(ros::NodeHandle &nh) noexcept
    : nh_(nh),
      contexts_(){

  // Creating the Webcam context
  auto active_webcam = false;
  nh_.getParam("/provider_vision/active_webcam", active_webcam);
  if (active_webcam) {
    contexts_.push_back(std::make_shared<WebcamContext>());
  }

  // Creating the DC1394 context
  std::vector<std::string> camera_names_dc1394;
  nh_.getParam("/provider_vision/active_dc1394", camera_names_dc1394);
  if (!camera_names_dc1394.empty()) {
    std::vector<CameraConfiguration> configurations;
    for (const auto &camera_dc : camera_names_dc1394) {
      configurations.push_back(CameraConfiguration(nh_, camera_dc));
    }
    contexts_.push_back(std::make_shared<DC1394Context>(configurations));
  }

  // Creating the GigE context
  std::vector<std::string> camera_names_gige;
  nh_.getParam("/provider_vision/active_gige", camera_names_gige);
  if (!camera_names_gige.empty()) {
    std::vector<CameraConfiguration> configurations;
    for (const auto &camera_gige : camera_names_gige) {
      configurations.push_back(CameraConfiguration(nh_, camera_gige));
    }
    contexts_.push_back(std::make_shared<GigeContext>(configurations));
  }
  // Creating the files context
  contexts_.push_back(std::make_shared<FileContext>());


  // Setting the callbacks
  server_.setCallback(boost::bind(&MediaManager::CallBackDynamicReconfigure, this, _1, _2));

  get_available_camera_ = nh_.advertiseService(kRosNodeName + "get_available_camera", &MediaManager::GetAvailableCameraCallback, this);
  start_stop_media_ = nh_.advertiseService(kRosNodeName + "start_stop_camera", &MediaManager::StartStopMediaCallback, this);
  set_camera_feature_= nh_.advertiseService(kRosNodeName + "set_camera_feature", &MediaManager::SetCameraFeatureCallback, this);
  get_camera_feature_ = nh_.advertiseService(kRosNodeName + "get_camera_feature", &MediaManager::GetCameraFeatureCallback, this);
}

//------------------------------------------------------------------------------
//
MediaManager::~MediaManager() {
  for (auto &elem : contexts_) {
    elem->CloseContext();
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
BaseMedia::Ptr MediaManager::GetMedia(const std::string &name) const {
  BaseMedia::Ptr media(nullptr);
  for (const auto &context : contexts_) {
    if (context->ContainsMedia(name)) {
      media = context->GetMedia(name);
    }
  }
  return media;
}

//------------------------------------------------------------------------------
//
std::vector<std::string> MediaManager::GetAllMediasName() const {
  std::vector<std::string> medias;

  for (auto &context : contexts_) {
    auto context_media_list = context->GetMediaList();
    for (const auto &media : context_media_list) {
      medias.push_back(media->GetName());
    }
  }
  return medias;
}

//------------------------------------------------------------------------------
//
bool MediaManager::SetCameraFeature(const std::string &media_name,
                                    const std::string &feature,
                                    const boost::any &value) {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context) {
    return context->SetFeature(GetFeatureFromName(feature), media_name, value);
  } else {
    ROS_INFO("MediaManager: Context not found for this media");
    return false;
  }
}

//------------------------------------------------------------------------------
//
bool MediaManager::GetCameraFeature(const std::string &media_name,
                                    const std::string &feature,
                                    boost::any &value) const {
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (context) {
    return context->GetFeature(GetFeatureFromName(feature), media_name, value);
  } else {
    ROS_INFO("MediaManager: Context not found for this media");
    return false;
  }
}

//------------------------------------------------------------------------------
//
BaseContext::Ptr MediaManager::GetContextFromMedia(
    const std::string &name) const {
  BaseContext::Ptr context_ptr(nullptr);
  for (auto &context : contexts_) {
    if (context->ContainsMedia(name)) {
      context_ptr = context;
    }
  }
  return context_ptr;
}

//------------------------------------------------------------------------------
//
BaseCamera::Feature MediaManager::GetFeatureFromName(
    const std::string &name) const {
  if (name == "SHUTTER_AUTO") {
    return BaseCamera::Feature::SHUTTER_AUTO;
  } else if (name == "SHUTTER") {
    return BaseCamera::Feature::SHUTTER_VALUE;
  } else if (name == "WHITE_BALANCE_AUTO") {
    return BaseCamera::Feature::WHITE_BALANCE_AUTO;
  } else if (name == "WHITE_BALANCE_RED") {
    return BaseCamera::Feature::WHITE_BALANCE_RED_VALUE;
  } else if (name == "WHITE_BALANCE_BLUE") {
    return BaseCamera::Feature::WHITE_BALANCE_BLUE_VALUE;
  } else if (name == "WHITE_BALANCE_GREEN") {
    return BaseCamera::Feature::WHITE_BALANCE_GREEN_VALUE;
  } else if (name == "FRAMERATE") {
    return BaseCamera::Feature::FRAMERATE_VALUE;
  } else if (name == "GAIN_AUTO") {
    return BaseCamera::Feature::GAIN_AUTO;
  } else if (name == "GAIN") {
    return BaseCamera::Feature::GAIN_VALUE;
  } else if (name == "EXPOSURE_AUTO") {
    return BaseCamera::Feature::EXPOSURE_AUTO;
  } else if (name == "EXPOSURE") {
    return BaseCamera::Feature::EXPOSURE_VALUE;
  } else if (name == "GAMMA") {
    return BaseCamera::Feature::GAMMA_VALUE;
  } else if (name == "SATURATION") {
    return BaseCamera::Feature::SATURATION_VALUE;
  } else if (name == "AUTOBRIGHTNESS_AUTO") {
    return BaseCamera::Feature::AUTOBRIGHTNESS_AUTO;
  } else if (name == "AUTOBRIGHTNESS_TARGET") {
    return BaseCamera::Feature::AUTOBRIGHTNESS_TARGET;
  } else if (name == "AUTOBRIGHTNESS_VARIATION") {
    return BaseCamera::Feature::AUTOBRIGHTNESS_VARIATION;
  } else if (name == "WHITE_BALANCE_EXECUTE") {
    return BaseCamera::Feature::WHITE_BALANCE_EXECUTE;
  } else {
    ROS_ERROR("MediaManager: No feature with this name");
    return BaseCamera::Feature::INVALID_FEATURE;
  }
}

//------------------------------------------------------------------------------
//
void MediaManager::CallBackDynamicReconfigure(
    provider_vision::Camera_Parameters_Config &config, uint32_t level) {
  (void)level;
  if (IsContextValid("bottom_gige")) {
    UpdateIfChanged("bottom_gige", "AUTOBRIGHTNESS_AUTO",
                    old_config_.bottom_gige_auto_brightness_auto,
                    config.bottom_gige_auto_brightness_auto);
    UpdateIfChanged("bottom_gige", "AUTOBRIGHTNESS_TARGET",
                    old_config_.bottom_gige_auto_brightness_target,
                    config.bottom_gige_auto_brightness_target);
    UpdateIfChanged("bottom_gige", "AUTOBRIGHTNESS_VARIATION",
                    old_config_.bottom_gige_auto_brightness_variation,
                    config.bottom_gige_auto_brightness_variation);
    UpdateIfChanged("bottom_gige", "EXPOSURE_AUTO",
                    old_config_.bottom_gige_exposure_auto,
                    config.bottom_gige_exposure_auto);
    UpdateIfChanged("bottom_gige", "EXPOSURE", old_config_.bottom_gige_exposure,
                    config.bottom_gige_exposure);
    UpdateIfChanged("bottom_gige", "GAIN_AUTO",
                    old_config_.bottom_gige_gain_auto,
                    config.bottom_gige_gain_auto);
    UpdateIfChanged("bottom_gige", "GAIN", old_config_.bottom_gige_gain,
                    config.bottom_gige_gain);
    // The sequence order for white balance is important
    // since in this case it acts as a execute. i.e, the blue,
    // green and red must be set after the white_balance_execute.
    UpdateIfChanged("bottom_gige", "WHITE_BALANCE_AUTO",
                    old_config_.bottom_gige_white_balance_execute,
                    config.bottom_gige_white_balance_execute);
    UpdateIfChanged("bottom_gige", "WHITE_BALANCE_BLUE",
                    old_config_.bottom_gige_white_balance_blue,
                    config.bottom_gige_white_balance_blue);
    UpdateIfChanged("bottom_gige", "WHITE_BALANCE_GREEN",
                    old_config_.bottom_gige_white_balance_green,
                    config.bottom_gige_white_balance_green);
    UpdateIfChanged("bottom_gige", "WHITE_BALANCE_RED",
                    old_config_.bottom_gige_white_balance_red,
                    config.bottom_gige_white_balance_red);
  }

  if (IsContextValid("bottom_guppy")) {
    UpdateIfChanged("bottom_guppy", "EXPOSURE_AUTO",
                    old_config_.bottom_guppy_exposure_auto,
                    config.bottom_guppy_exposure_auto);
    UpdateIfChanged("bottom_guppy", "EXPOSURE",
                    old_config_.bottom_guppy_exposure,
                    config.bottom_guppy_exposure);
    UpdateIfChanged("bottom_guppy", "GAIN_AUTO",
                    old_config_.bottom_guppy_gain_auto,
                    config.bottom_guppy_gain_auto);
    UpdateIfChanged("bottom_guppy", "GAIN", old_config_.bottom_guppy_gain,
                    config.bottom_guppy_gain);
    UpdateIfChanged("bottom_guppy", "SHUTTER_AUTO",
                    old_config_.bottom_guppy_shutter_auto,
                    config.bottom_guppy_shutter_auto);
    UpdateIfChanged("bottom_guppy", "SHUTTER", old_config_.bottom_guppy_shutter,
                    config.bottom_guppy_shutter);
    UpdateIfChanged("bottom_guppy", "WHITE_BALANCE_BLUE",
                    old_config_.bottom_guppy_white_balance_blue,
                    config.bottom_guppy_white_balance_blue);
    UpdateIfChanged("bottom_guppy", "WHITE_BALANCE_AUTO",
                    old_config_.bottom_guppy_white_balance_auto,
                    config.bottom_guppy_white_balance_auto);
    UpdateIfChanged("bottom_guppy", "WHITE_BALANCE_RED",
                    old_config_.bottom_guppy_white_balance_red,
                    config.bottom_guppy_white_balance_red);
  }

  if (IsContextValid("front_guppy")) {
    UpdateIfChanged("front_guppy", "EXPOSURE_AUTO",
                    old_config_.front_guppy_exposure_auto,
                    config.front_guppy_exposure_auto);
    UpdateIfChanged("front_guppy", "EXPOSURE", old_config_.front_guppy_exposure,
                    config.front_guppy_exposure);
    UpdateIfChanged("front_guppy", "GAIN_AUTO",
                    old_config_.front_guppy_gain_auto,
                    config.front_guppy_gain_auto);
    UpdateIfChanged("front_guppy", "GAIN", old_config_.front_guppy_gain,
                    config.front_guppy_gain);
    UpdateIfChanged("front_guppy", "SHUTTER_AUTO",
                    old_config_.front_guppy_shutter_auto,
                    config.front_guppy_shutter_auto);
    UpdateIfChanged("front_guppy", "SHUTTER", old_config_.front_guppy_shutter,
                    config.front_guppy_shutter);
    UpdateIfChanged("front_guppy", "WHITE_BALANCE_BLUE",
                    old_config_.front_guppy_white_balance_blue,
                    config.front_guppy_white_balance_blue);
    UpdateIfChanged("front_guppy", "WHITE_BALANCE_AUTO",
                    old_config_.front_guppy_white_balance_auto,
                    config.front_guppy_white_balance_auto);
    UpdateIfChanged("front_guppy", "WHITE_BALANCE_RED",
                    old_config_.front_guppy_white_balance_red,
                    config.front_guppy_white_balance_red);
  }

  old_config_ = config;
}

void MediaManager::UpdateIfChanged(std::string camera_name,
                                   std::string feature_name, bool &old_state,
                                   bool &state) {
  if (state != old_state) {
    SetCameraFeature(camera_name, feature_name, boost::any(state));
    ROS_INFO("Setting %s on %s to %i", feature_name.c_str(),
             camera_name.c_str(), state);
  }

  atlas::MilliTimer::Sleep(100);

  boost::any real_state;
  GetCameraFeature(camera_name, feature_name, real_state);
  try {
    state = boost::any_cast<bool>(real_state);
  } catch (std::exception &e) {
    ROS_INFO("Trouble casting to bool");
  }
}

void MediaManager::UpdateIfChanged(std::string camera_name,
                                   std::string feature_name, double &old_value,
                                   double &value) {
  if (value != old_value) {
    SetCameraFeature(camera_name, feature_name, boost::any(value));
    ROS_INFO("Setting %s on %s to %.2f", feature_name.c_str(),
             camera_name.c_str(), value);
  }

  atlas::MilliTimer::Sleep(100);

  boost::any real_state;
  GetCameraFeature(camera_name, feature_name, real_state);
  try {
    value = boost::any_cast<double>(real_state);
  } catch (std::exception &e) {
    ROS_INFO("Trouble casting to double");
  }
}

void MediaManager::UpdateIfChanged(std::string camera_name,
                                   std::string feature_name, int &old_value,
                                   int &value) {
  if (value != old_value) {
    SetCameraFeature(camera_name, feature_name, boost::any(value));
    ROS_INFO("Setting %s on %s to %i", feature_name.c_str(),
             camera_name.c_str(), value);
  }

  atlas::MilliTimer::Sleep(100);

  boost::any real_state;
  GetCameraFeature(camera_name, feature_name, real_state);

  try {
    value = boost::any_cast<int>(real_state);
    if (value == -1) value = old_value;
  } catch (std::exception &e) {
    ROS_INFO("Trouble casting to int");
  }
}

bool MediaManager::IsContextValid(const std::string &name) {
  for (auto &elem : contexts_) {
    if (elem->ContainsMedia(name)) {
      return true;
    }
  }
  return false;
}

bool MediaManager::GetAvailableCameraCallback(
    provider_vision::get_available_cameraRequest &request,
    provider_vision::get_available_cameraResponse &response)
{
  response.available_media = GetAllMediasName();
  return true;
}

bool MediaManager::StartStopMediaCallback(
    provider_vision::start_stop_mediaRequest &request,
    provider_vision::start_stop_mediaResponse &response)
{
  // This function need to be cleaned.
  if( request.action == request.START)
  {
    response.action_accomplished = (uint8_t)StartStreaming(request.camera_name);

  }else if (request.action == request.STOP)
  {
    response.action_accomplished = (uint8_t)StopStreaming(request.camera_name);
  }else
  {
    ROS_ERROR("Action is neither stop or start. Cannot proceed.");
    response.action_accomplished = (uint8_t)false;
  }
  return true;
}

bool MediaManager::StopStreaming(const std::string &camera_name) {
  bool action_accomplished = false;
  MediaStreamer::Ptr streamer = GetMediaStreamer(camera_name);
  if (streamer) {
    // Do not use the result variables, since the remove will do the job.
    RemoveMediaStreamer(camera_name);
    action_accomplished = true;
    ROS_INFO("Media is stopped.");
  } else {
    ROS_ERROR("Media streamer could not be found");
  }
  return action_accomplished;
}

bool MediaManager::StartStreaming(const std::string &media_name) {// If the media is already streaming, return the streamer
  bool action_accomplished = false;

  if (IsMediaStreaming(media_name)) {
    return action_accomplished;
  }
  // Find which context to owns the media.
  BaseContext::Ptr context = GetContextFromMedia(media_name);
  if (!context) {
    ROS_ERROR("The requested media is not part of a context.");
    return action_accomplished;
  }
  // The context set the media to stream
  if (!context->OpenMedia(media_name)) {
    ROS_ERROR("The media cannot start streaming.");
    return action_accomplished;
  }

  // Get the media to create a streamer
  BaseMedia::Ptr media = context->GetMedia(media_name);
  if (!media) {
    ROS_ERROR("Camera failed to be created");
    return action_accomplished;
  }

  std::string new_name = FormatNameForTopic(media_name);

  auto streamer = std::make_shared<MediaStreamer>(media, nh_, kRosNodeName + new_name, 30);
  if (!streamer) {
    ROS_ERROR("Streamer failed to be created");
    return action_accomplished;
  }
  // Keep it in the list of Streaming media
  AddMediaStreamer(streamer);
  action_accomplished = true;
  return action_accomplished;
}

std::string MediaManager::FormatNameForTopic(const std::string &media_name) const {
// if the media is valid, create the streamer
  // But why not simply use the name from the cam? well if it is a file, it has a . in it (.png) and this
  // crashes the program : Character [.] at element [27] is not valid in Graph Resource Name
  // [/home/jeremie/Pictures/test.png].  Valid characters are a-z, A-Z, 0-9, / and _.
  std::string new_name = media_name;
  new_name.erase(std::remove(new_name.begin(), new_name.end(), '.'), new_name.end());
  return new_name;
}

//------------------------------------------------------------------------------
//
bool MediaManager::GetCameraFeatureCallback(
    provider_vision::get_camera_feature::Request &rqst,
    provider_vision::get_camera_feature::Response &rep) {
  boost::any feature_value;
  GetCameraFeature(rqst.camera_name, rqst.camera_feature,
                   feature_value);
  std::stringstream ss;
  if (feature_value.type() == typeid(bool)) {
    rep.feature_type = rqst.FEATURE_BOOL;
    ss << boost::any_cast<bool>(feature_value);
  } else if (feature_value.type() == typeid(int)) {
    rep.feature_type = rqst.FEATURE_INT;
    ss << boost::any_cast<int>(feature_value);
  } else if (feature_value.type() == typeid(double)) {
    rep.feature_type = rqst.FEATURE_DOUBLE;
    ss << boost::any_cast<double>(feature_value);
  } else {
    ROS_ERROR("The feature type is not supported.");
    return false;
  }
  rep.feature_value = ss.str();
  return true;
}

//------------------------------------------------------------------------------
//
bool MediaManager::SetCameraFeatureCallback(
    provider_vision::set_camera_feature::Request &rqst,
    provider_vision::set_camera_feature::Response &rep) {
  if (rqst.feature_type == rqst.FEATURE_BOOL) {
    boost::any value = boost::lexical_cast<bool>(rqst.feature_value);
    SetCameraFeature(rqst.camera_name, rqst.camera_feature, value);
  } else if (rqst.feature_type == rqst.FEATURE_DOUBLE) {
    boost::any value = boost::lexical_cast<double>(rqst.feature_value);
    SetCameraFeature(rqst.camera_name, rqst.camera_feature, value);
  } else if (rqst.feature_type == rqst.FEATURE_INT) {
    boost::any value = boost::lexical_cast<int>(rqst.feature_value);
    SetCameraFeature(rqst.camera_name, rqst.camera_feature, value);
  } else {
    ROS_ERROR("The feature type is not supported.");
    return false;
  }
  return true;
}

}  // namespace provider_vision
