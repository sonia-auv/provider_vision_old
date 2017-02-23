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

#include <provider_vision/media/context/topic_context.h>

namespace provider_vision {

//==============================================================================
// C / D T O R S   S E C T I O N
    const char *TopicContext::DRIVER_TAG = "Topic context";
//------------------------------------------------------------------------------
//
    TopicContext::TopicContext() : BaseContext() {}

//------------------------------------------------------------------------------
//
    TopicContext::~TopicContext() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
    void TopicContext::CloseContext() { media_list_.clear(); }

//------------------------------------------------------------------------------
//
    bool TopicContext::OpenMedia(const std::string &name) {
        BaseMedia::Ptr media = GetMedia(name);

        // If we did not find the media, this means it is not open, so we need to
        // open it.
        if (!media) {
                ImageTopic::Ptr topic(std::make_shared<ImageTopic>(name));
                topic->Open();
                media_list_.push_back(std::dynamic_pointer_cast<BaseMedia>(topic));
            } else {
                ROS_ERROR("%s Not my media type", DRIVER_TAG);
                return false;
            }
        return true;
    }

//------------------------------------------------------------------------------
//
    bool TopicContext::CloseMedia(const std::string &name) {
        auto topic = GetMedia(name);
        if (topic) {
            // Here, we do not look for success, erase media will delete it anyway.
            topic->Close();
            EraseMedia(name);
            return true;
        }
        ROS_ERROR("%s Media not found", DRIVER_TAG);
        return false;
    }

//------------------------------------------------------------------------------
//
    bool TopicContext::StartStreamingMedia(const std::string &name) {
        auto topic = GetMedia(name);
        // File might not be running (new video)
        if (topic) {
            return topic->StartStreaming();
        } else {
            return OpenMedia(name);
        }
    }

//------------------------------------------------------------------------------
//
    bool TopicContext::StopStreamingMedia(const std::string &name) {
        auto topic = GetMedia(name);
        if (topic) {
            return topic->StopStreaming();
        }
        ROS_ERROR("%s Media not found", DRIVER_TAG);
        return false;
    }

//------------------------------------------------------------------------------
//
    bool TopicContext::ContainsMedia(const std::string &nameMedia) const {
        bool result = false;
        // This function might be called when creating a video/image file, so we
        // won't have in our media BUT it is still ours.
        //TODO: check if name is topic
        if (GetMedia(nameMedia)) {
            result = true;
        }
        return result;
    }

//------------------------------------------------------------------------------
//
    bool TopicContext::SetFeature(const BaseCamera::Feature &feat,
                                 const std::string &name, const boost::any &val) {
        // Here we make a choice to return false, since if you try to set a parameter
        // on a file, there is something wrong...
        return false;
    }

//------------------------------------------------------------------------------
//
    bool TopicContext::GetFeature(const BaseCamera::Feature &feat,
                                 const std::string &name, boost::any &val) const {
        // Here we make a choice to return false, since if you try to set a parameter
        // on a file, there is something wrong...
        return false;
    }

//------------------------------------------------------------------------------
//
    void TopicContext::Run() {}

//------------------------------------------------------------------------------
//
    bool TopicContext::WatchDogFunc() { return true; }

}  // namespace provider_vision
