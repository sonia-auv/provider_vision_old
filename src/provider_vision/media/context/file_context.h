/**
 * \file	VideoFileContext.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	10/03/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAM_DRIVER_MEDIA_H_
#define VISION_SERVER_CAM_DRIVER_MEDIA_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <opencv2/opencv.hpp>
#include "provider_vision/media/camera/base_media.h"
#include "provider_vision/media/context/base_context.h"
#include "provider_vision/config.h"
#include "provider_vision/media/camera/video_file.h"

namespace vision_server {

  //==============================================================================
  // C L A S S E S

  /**
   * Class managing medias from a hard drive.
   *
   * Allow to load a video or a picture from a hard drive onto the vision server.
   * The creation of a static media is done during the execution in the method
   * StartCamera().
   * We Currently use _live_camera_list because the responsability of camera_list
   * is to list all the available camera. As they are videos, they are all and
   * always available.
   * Therefore, calling GetCameraList() will return the list of videos from
   * _live_camera_list.
   */
  class VideoFileContext : public BaseContext {
  public:
    const char *DRIVER_TAG;

    //==========================================================================
    // T Y P E D E F   A N D   E N U M

    enum class MediaType { IMAGE, VIDEO, NONE };

    //==========================================================================
    // P U B L I C   C / D T O R S

    VideoFileContext();

    virtual ~VideoFileContext();

    //==========================================================================
    // P U B L I C   M E T H O D S

    void InitContext(const std::vector<CameraConfiguration> &cam_configuration_lists) override;

    void CloseContext() override;

    bool StartCamera(const std::string &name) override;

    bool StopCamera(const std::string &name) override;

    void SetFeature(BaseCamera::Feature feat, const std::string &name,
                    float val) override;

    void GetFeature(BaseCamera::Feature feat, const std::string &name,
                    float &val) const override;

    bool IsMyCamera(const std::string &nameMedia) const override;

    void run() override;

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

  //==============================================================================
  // I N L I N E   F U N C T I O N S   D E F I N I T I O N S

}  // namespace vision_server

#endif  // VISION_SERVER_CAM_DRIVER_MEDIA_H_
