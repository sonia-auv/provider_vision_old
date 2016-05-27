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

#ifndef PROVIDER_VISION_SERVER_MEDIA_MANAGER_H_
#define PROVIDER_VISION_SERVER_MEDIA_MANAGER_H_

#include <provider_vision/media/camera/base_media.h>
#include <memory>
#include <string>
#include <vector>
#include "provider_vision/media/camera/base_camera.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/media/media_streamer.h"

namespace provider_vision {

class MediaManager {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MediaManager>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit MediaManager(const ros::NodeHandle &nh) noexcept;

  ~MediaManager() noexcept;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void OpenMedia(const std::string &media_name);

  void CloseMedia(const std::string &media_name);

  /**
   * Start the acquisition of the images on the given media.
   *
   * By default, the media streamer will not be streaming, if the stream flag
   * is set to true, it will start a thread and notify all observers whenever
   * there is an image. See MediaStreamer class for more informations.
   */
  MediaStreamer::Ptr StartStreamingMedia(const std::string &media_name);

  void StopStreamingMedia(const std::string &media) noexcept;

  /**
   * Get the name of all existing medias in the system.
   *
   * If there is cameras pluged in the system, they will appear in this list.
   * If there is images or video files streaming, they will appear here as well.
   *
   * \return The name of all medias in the system.
   */
  std::vector<std::string> GetAllMediasName() const noexcept;

  /**
   * Get the number of all medias in the system.
   *
   * This will iterate through all media list in order to have the total count.
   *
   * \return The total count of all medias.
   */
  size_t GetAllMediasCount() const noexcept;

  /**
   * If the media is a camera, set the feature to a specific value.
   *
   * This will try to convert the media to a camera. If this work, calls the
   * method to set a feature on it. If it does not, this throws an exception.
   *
   * \param media_name The name of the media to set the feature to.
   * \param feature The feature to change the value of.
   * \param value The value to set on the given feature.
   */
  void SetCameraFeature(const std::string &media_name,
                        const std::string &feature, boost::any &value);

  /**
   * If the media is a camera, get the value of the given feature.
   *
   * This will try to convert the media to a camera. If this work, calls the
   * method to ge the value of a feature on it. If it does not, this throws
   * an exception.
   *
   * \param media_name The name of the media to set the feature to.
   * \param feature The feature to change the value of.
   */
  void GetCameraFeature(const std::string &media_name,
                        const std::string &feature, boost::any &value) const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S
  /**
   * Return true if a media streamer has been created for the given media.
   *
   * This will compare the value of the media name with all the media streamer
   * in the system and this will return true if there is a match, false if not.
   *
   * \param name The media name to check.
   * \return True if the media is streaming.
   */
  bool IsMediaStreaming(const std::string &name);

  void StopStreamingMedia(const MediaStreamer::Ptr &streamer) noexcept;

  BaseMedia::Ptr GetMedia(const std::string &name) const noexcept;

  BaseContext::Ptr GetContextFromMedia(const std::string &name) const;

  BaseCamera::Feature GetFeatureFromName(const std::string &name) const;

  MediaStreamer::Ptr GetMediaStreamer(const std::string &name);

  void AddMediaStreamer(MediaStreamer::Ptr media_streamer);

  void RemoveMediaStreamer(const std::string &name);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;

  std::vector<BaseContext::Ptr> contexts_;

  std::vector<MediaStreamer::Ptr> media_streamers_;
};

//-----------------------------------------------------------------------------
//
inline MediaStreamer::Ptr MediaManager::GetMediaStreamer(
    const std::string &name) {
  MediaStreamer::Ptr media_ptr(nullptr);

  for (const auto &elem : media_streamers_) {
    if (elem->GetMediaName().compare(name) == 0) {
      media_ptr = elem;
    }
  }
  return media_ptr;
}

//-----------------------------------------------------------------------------
//
inline void MediaManager::AddMediaStreamer(MediaStreamer::Ptr media_streamer) {
  media_streamers_.push_back(media_streamer);
}

//-----------------------------------------------------------------------------
//
inline void MediaManager::RemoveMediaStreamer(const std::string &name) {
  for (auto elem = media_streamers_.begin(); elem != media_streamers_.end();
       elem++) {
    if ((*elem)->GetMediaName().compare(name) == 0) {
      media_streamers_.erase(elem);
      return;
    }
  }
}

//-------------------------------------------------------------------------
//
inline bool MediaManager::IsMediaStreaming(const std::string &name) {
  for (const auto &elem : media_streamers_) {
    if (name.compare(elem->GetMediaName()) == 0) {
      return true;
    }
  }
  return false;
}

}  // namespace provider_vision

#endif  // PROVIDER_VISION_SERVER_MEDIA_MANAGER_H_
