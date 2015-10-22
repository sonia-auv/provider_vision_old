/**
 * \file  performance_evaluator_test.cc
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_vision/algorithm/performance_evaluator.h>

TEST(PerformanceEvaluator, AllTest) {
   PerformanceEvaluator pe;

  pe.UpdateStartTime();
  sleep(1);
  double execTime = pe.GetExecTimeSec();
  ASSERT_TRUE(execTime > 0.1f);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
