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

#include <thread>

#include "provider_vision/media/media_streamer.h"

namespace provider_vision {


//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MediaStreamer::MediaStreamer(BaseMedia::Ptr cam, ros::NodeHandle &node_handle, const std::string &topic_name,
                             int artificialFrameRateMs)
    : media_(cam),
      stop_thread_(false),
      thread_(std::bind(&MediaStreamer::BroadcastThread, this)),
      image_publisher_(),
      it_(node_handle),
      frame_rate_(artificialFrameRateMs)
{
  // Create the broadcast topic.
  image_publisher_ = it_.advertise(topic_name, 100);
}

//------------------------------------------------------------------------------
//
MediaStreamer::~MediaStreamer() {
  // Set the flag to stop the thread and wait for it to stop
  stop_thread_ = true;
  thread_.join();
  // Shutdown the topic
  image_publisher_.shutdown();
  ROS_INFO("%s closed", media_->GetName().c_str());
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MediaStreamer::BroadcastThread() {
  // Starting a timer for timing the acquisition of the image from the media.
  atlas::MilliTimer timer;
  timer.Start();
  cv::Mat image;
  cv_bridge::CvImage ros_image;

  while (!stop_thread_) {
    bool result = false;
    try
    {
      result = media_->NextImage(image);

      // We gotta a image
      if (!image.empty() && result) {
        //publish image
        ros_image.image = image;
        ros_image.encoding = sensor_msgs::image_encodings::BGR8;
        image_publisher_.publish(ros_image.toImageMsg());
        // Reset the timer for next acquisition
        timer.Reset();

      } else {
        // if we have received any images in 1 sec, there is a problem
        if( timer.Seconds() > 1) {
          ROS_ERROR("Media streamer %s haven't broadcast new image for 1 sec.",
                    media_->GetName().c_str());
          timer.Reset();
        }
      }

      // For the files, we set a fixed framerate at 15 fps
      if (media_->HasArtificialFramerate()) {
        usleep(int(1000000.0f * (1.0f/frame_rate_)));
      }
    }catch (std::exception &e)
    {
      ROS_ERROR("Exception caught in thread of %s : %s",
                media_->GetName().c_str(), e.what());
    }
  }
}
}  // namespace provider_vision
