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

#ifndef PROVIDER_VISION_MEDIA_CONTEXT_FILE_CONTEXT_H_
#define PROVIDER_VISION_MEDIA_CONTEXT_FILE_CONTEXT_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "provider_vision/config.h"
#include "provider_vision/media/camera/image_file.h"
#include "provider_vision/media/camera/video_file.h"
#include "provider_vision/media/context/base_context.h"

namespace provider_vision {

/**
 * Class managing medias from a hard drive.
 *
 * Allow to load a video or a picture from a hard drive onto the vision server.
 * The creation of a static media is done during the execution in the method
 * StartMedia().
 * We Currently use _live_camera_list because the responsability of camera_list
 * is to list all the available camera. As they are videos, they are all and
 * always available.
 * Therefore, calling GetCameraList() will return the list of videos from
 * _live_camera_list.
 */
class FileContext : public BaseContext {
 public:
  static const std::string DRIVER_TAG;

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FileContext>;

  enum class MediaType { IMAGE, VIDEO, NONE };

  //==========================================================================
  // P U B L I C   C / D T O R S

  FileContext() noexcept;

  virtual ~FileContext();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void CloseContext() override;

  void OpenMedia(const std::string &name) override;

  void CloseMedia(const std::string &name) override;

  void StartStreamingMedia(const std::string &name) override;

  void StopStreamingMedia(const std::string &name) override;

  virtual void GetFeature(const BaseCamera::Feature &feat,
                          const std::string &name, boost::any &val) const override;

  virtual void SetFeature(const BaseCamera::Feature &feat, const std::string
  &name, boost::any &val) override;

  bool ContainsMedia(const std::string &nameMedia) const override;

  void Run() override;

  bool WatchDogFunc() override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Return the type of a specific media passed in parameters.
   * The parameter can be either en image or a video
   */
  virtual MediaType GetMediaType(const std::string &nameMedia) const;
};

}  // namespace provider_vision

#endif  // PROVIDER_VISION_CAM_DRIVER_MEDIA_H_
