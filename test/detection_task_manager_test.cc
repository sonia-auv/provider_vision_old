/**
 * \file  detection_task_manager.cc
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "provider_vision/utils/config.h"
#include "provider_vision/server/media_manager.h"

using namespace vision_server;


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
