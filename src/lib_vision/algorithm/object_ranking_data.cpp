/**
 * \file	object_ranking_data.cpp
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_vision/algorithm/object_ranking_data.h>

//=============================================================================
//=============================================================================
//=============================================================================
//=========================== UNIT TEST AREA ==================================
//=============================================================================
//=============================================================================
//=============================================================================
#include <TCUnitTest.h>

TC_DEFINE_UNIT_TEST(ObjectRankingDataUT) {
  printf("Starting unit test on ObjectRankingData");

  ObjectRankingData obj;
  obj.SetAreaRank(1);
  obj.SetLengthRank(2);
  TC_TEST_FAIL("Setter ok", obj.GetAreaRank() == 1 && obj.GetLengthRank() == 2);

  printf("System all clear and good to go");
  return true;
}
TC_END_UNIT_TEST(ObjectRankingDataUT);
