/**
 * \file	CameraID.h
 * \author	Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \date	18/10/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef VISION_SERVER_CAMERA_ID_H_
#define VISION_SERVER_CAMERA_ID_H_

//==============================================================================
// I N C L U D E   F I L E S

#include <stdint.h>
#include <string>
#include "media/cam_undistord_matrices.h"

namespace vision_server {

//==============================================================================
// C L A S S E S

/**
 * CameraID is a little class to handle the link between GUID and name.
 * We can then identify a camera by its name but the system uses its GUID.
 */
class CameraID {
 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  CameraID();

  CameraID(std::string name, uint64_t guid);

  virtual ~CameraID();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Set(std::string name, uint64_t guid);

  const std::string GetName() const;

  const uint64_t GetGUID() const;

  const char *GetFullName() const;

  // TODO
  // Fix this. Hack to prevent to much refactoring before competition.
  // Since the CameraID is given to every one for the cameras handling,
  // add the undistord matrices as members so it can piggy back
  // the variable passing. However, it should be a separate object, part
  // of a config object that will be pass instead of CameraID
  CameraUndistordMatrices _camUndistordMatrices;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::string _name;

  uint64_t _guid;

  std::string _full_id_string;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void CameraID::Set(std::string name, uint64_t guid) {
  _name = name;
  _guid = guid;
  _full_id_string = name;
  _full_id_string += "," + std::to_string(_guid);
}

//------------------------------------------------------------------------------
//
inline const std::string CameraID::GetName() const { return _name; }

//------------------------------------------------------------------------------
//
inline const uint64_t CameraID::GetGUID() const { return _guid; }

//------------------------------------------------------------------------------
//
inline const char *CameraID::GetFullName() const {
  return _full_id_string.c_str();
}

}  // namespace vision_server

#endif  // VISION_SERVER_CAMERA_ID_H_
