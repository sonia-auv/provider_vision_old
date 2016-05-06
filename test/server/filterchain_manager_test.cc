/**
 * \file  filterchain_manager_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  22/10/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "provider_vision/server/filterchain_manager.h"

ros::NodeHandle *nhp;

TEST(FeatureFactory, AllTest) {
  provider_vision::FilterchainManager fcmgr;
  auto filterchains = fcmgr.InstanciateAllFilterchains();
  for(auto &filterchain : filterchains) {
  filterchain->Serialize();
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "provider_vision");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
