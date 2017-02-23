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

#include "provider_vision/media/camera/image_topic.h"
#include <string>
#include <vector>

namespace provider_vision {


//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
    ImageTopic::ImageTopic(const std::string &topic) noexcept
            : BaseMedia(topic), topic_(topic), it_(nh_) {}

//------------------------------------------------------------------------------
//
    ImageTopic::~ImageTopic() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
    bool ImageTopic::Open() {
        image_transport::Subscriber sub =
                it_.subscribe(topic_, 1, &ImageTopic::ImageCallback, this);
        status_ = Status::OPEN;
        return true;
    }

//------------------------------------------------------------------------------
//
    void ImageTopic::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

//------------------------------------------------------------------------------
//
    bool ImageTopic::Close() {
        status_ = Status::CLOSE;
        return true;
    }

//------------------------------------------------------------------------------
//
    bool ImageTopic::SetStreamingModeOn() {
        status_ = Status::STREAMING;
        return true;
    }

//------------------------------------------------------------------------------
//
    bool ImageTopic::SetStreamingModeOff() {
        status_ = Status::OPEN;
        return true;
    }

//------------------------------------------------------------------------------
//
    bool ImageTopic::NextImage(cv::Mat &image) {
        if (!image_.empty()) {
            image_.copyTo(image);
            return true;
        }
        return false;
    }

//------------------------------------------------------------------------------
//
    void ImageTopic::NextImageCopy(cv::Mat &image) { NextImage(image); }

}  // namespace provider_vision
