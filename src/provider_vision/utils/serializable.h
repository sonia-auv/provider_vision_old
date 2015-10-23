/**
 * \file	serializer.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	14/06/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <string>

#ifndef PROVIDER_VISION_UTILS_SERIALIZABLE_H_
#define PROVIDER_VISION_UTILS_SERIALIZABLE_H_

namespace vision_server {

/**
 * This class provide an interface for all serializer.
 *
 * It basically provide public method to load and save to a file the state of
 * an object. You will have to implement the method to save the state of an
 * object to a file and the method to load the object from the file.
 */
class Serializable {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  explicit Serializable(const std::string &filepath) : filepath_(filepath) {}

  virtual ~Serializable() {}

  //============================================================================
  // P U B L I C   M E T H O D S

  virtual bool Serialize() = 0;

  virtual bool Deserialize() = 0;

  //============================================================================
  // G E T T E R S   A N D   S E T T E R S

  const std::string &filepath() const;

  void set_filepath(const std::string &filepath);

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  std::string filepath_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline const std::string &Serializable::filepath() const { return filepath_; }

//------------------------------------------------------------------------------
//
inline void Serializable::set_filepath(const std::string &filepath) {
  filepath_ = filepath;
}

}  // namespace vision_server

#endif  // PROVIDER_VISION_UTILS_SERIALIZABLE_H_
