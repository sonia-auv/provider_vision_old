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

#ifndef PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_
#define PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_

#include <thread>
#include <mutex>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <lib_atlas/sys/timer.h>
#include "provider_vision/media/camera/base_media.h"


namespace provider_vision {

/**
 * Class responsible of acquiring an image from a device and broadcasting it to ROS.
 * It is basically a thread running and getting images from a media.
 */
class MediaStreamer {
public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MediaStreamer>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  /**
   * Artificial frame rate simulate a frame rate for video and images.
   * It will run the loop at this speed.
   */
  explicit MediaStreamer(BaseMedia::Ptr cam, ros::NodeHandle &node_handle, const std::string &topic_name,
                         int artificialFrameRateMs = 30);

  virtual ~MediaStreamer();

  std::string GetMediaName();

private:
  //==========================================================================
  // P R I V A T E   M E T H O D S
  void BroadcastThread();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  // Active media for the loop
  BaseMedia::Ptr media_;
  // Flag to stop the thread
  bool stop_thread_;
  // The thread for broadcasting an image
  std::thread thread_;
  // Necessary publisher for the image
  image_transport::Publisher image_publisher_;
  image_transport::ImageTransport it_;

  float frame_rate_;

};

inline std::string MediaStreamer::GetMediaName() {
  return media_->GetName();
}


}



#endif  // PROVIDER_VISION_MEDIA_MEDIA_STREAMER_H_
