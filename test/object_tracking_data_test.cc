/**
 * \file	object_tracking_data_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/object_tracking_data.h>

TEST(AITrainer, AllTest) {
  printf("Starting unit test on ObjectTrackingData");
  OBjectTrackingData tkData;
  tkData.SetPresenceCount(2);
  ASSERT_TRUE(tkData.GetPresenceCount() == 2);
  printf("System all clear and good to go");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
