/**
 * \file	object_ranking_data.h
 * \author  Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \date	1/01/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_FILTER_RANKING_DATA_H_
#define VISION_FILTER_RANKING_DATA_H_

// Basic container class. It holds information about the ranking
// of certain caracteristic of an object compare to the other.
// In other word, it contains the position resulting of the sorting
// by ObjectRanker.
class ObjectRankingData {
 public:
  ObjectRankingData() : _area_rank(0.0f), _length_rank(0.0f){};

  virtual ~ObjectRankingData(){};

  // Rank are grade from 0 to 1, 0 being the last, 1 being the first
  void SetAreaRank(float rank);

  void SetLengthRank(float rank);

  float GetAreaRank();

  float GetLengthRank();

 private:
  float _area_rank;
  float _length_rank;
};
//=============================================================================
// 	INLINE METHOD IMPLEMENTATION

//-----------------------------------------------------------------------------
//
inline void ObjectRankingData::SetAreaRank(float rank) { _area_rank = rank; }

//-----------------------------------------------------------------------------
//
inline void ObjectRankingData::SetLengthRank(float rank) {
  _length_rank = rank;
}

//-----------------------------------------------------------------------------
//
inline float ObjectRankingData::GetAreaRank() { return _area_rank; }

//-----------------------------------------------------------------------------
//
inline float ObjectRankingData::GetLengthRank() { return _length_rank; }

#endif  // RANKING_DATA_H_
