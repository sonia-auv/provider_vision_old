/**
 * \file	object_ranking_data.h
 * \author	Jérémie St-Jules Prévôt <jeremie.st.jules.prevost@gmail.com>
 * \author  Pierluc Bédard <pierlucbed@gmail.com>
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIB_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_
#define LIB_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_

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

#endif  // LIB_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_
