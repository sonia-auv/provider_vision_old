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

#include <memory>

// Basic container class. It holds information about the ranking
// of certain caracteristic of an object compare to the other.
// In other word, it contains the position resulting of the sorting
// by ObjectRanker.
class ObjectRankingData {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectRankingData>;

  //============================================================================
  // P U B L I C   C / D T O R S

  ObjectRankingData() : _area_rank(0.0f), _length_rank(0.0f){};

  virtual ~ObjectRankingData(){};

  //============================================================================
  // P U B L I C   M E T H O D S

  // Rank are grade from 0 to 1, 0 being the last, 1 being the first
  void SetAreaRank(float rank);

  void SetLengthRank(float rank);

  float GetAreaRank();

  float GetLengthRank();

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  float _area_rank;
  float _length_rank;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void ObjectRankingData::SetAreaRank(float rank) { _area_rank = rank; }

//------------------------------------------------------------------------------
//
inline void ObjectRankingData::SetLengthRank(float rank) {
  _length_rank = rank;
}

//------------------------------------------------------------------------------
//
inline float ObjectRankingData::GetAreaRank() { return _area_rank; }

//------------------------------------------------------------------------------
//
inline float ObjectRankingData::GetLengthRank() { return _length_rank; }

#endif  // LIB_VISION_ALGORITHM_OBJECT_RANKING_DATA_H_
