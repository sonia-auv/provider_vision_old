/**
 * \file  configuration_handler_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Jeremie St-Jules-Prevost <jeremie.st.jules.prevost@gmail.com>
 * \date  28/06/2015
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include "provider_vision/media/configuration_handler.h"
#include "provider_vision/media/camera_configuration.h"
#include "provider_vision/config.h"

using namespace provider_vision;

TEST(ConfigurationHandler, loading) {
  std::string file_path(kProjectPath + "test/camera_xml_config.xml");
  ConfigurationHandler cfHdl(file_path);
  std::vector<CameraConfiguration> cams_config = cfHdl.ParseConfiguration();

  // Camera 1
  CameraConfiguration camera(cams_config[0]);
  ASSERT_EQ(camera.GetName().compare("Front"), 0);
  ASSERT_EQ(camera.GetUndistortionMatricePath().compare(
                kProjectPath + "test/front_guppy_calibration.xml"),
            0);

  ASSERT_EQ(camera.GetGUID(), 0xFF);

  ASSERT_EQ(camera.GetBoolean("AutoWB"), true);
  ASSERT_EQ(camera.GetString("AutoWB").compare("True"), 0);
  ASSERT_EQ(camera.GetFloat("Shutter"), 1.0);
  ASSERT_EQ(camera.GetInteger("Gain"), 32);

  camera = cams_config[1];

  ASSERT_EQ(camera.GetName().compare("Bottom"), 0);
  ASSERT_EQ(camera.GetUndistortionMatricePath().compare(""), 0);
  ASSERT_EQ(camera.GetGUID(), 0x11);

  ASSERT_EQ(camera.GetBoolean("AutoWB"), false);
  ASSERT_EQ(camera.GetFloat("Shutter"), 2.0);
  ASSERT_EQ(camera.GetInteger("Gain"), 3);
}

TEST(ConfigurationHandler, saving) {
  std::string file_path(kProjectPath + "test/camera_xml_config_2.xml");
  ConfigurationHandler cfHdl(file_path);
  std::vector<CameraConfiguration> cams_config = cfHdl.ParseConfiguration();

  CameraConfiguration camConfig("Front");
  camConfig.SetGUID(0xFF);
  camConfig.AddConfiguration("conf1", "True");
  camConfig.AddConfiguration("conf2", "1.0");
  cams_config.push_back(camConfig);
  cfHdl.SaveConfiguration(cams_config);

  // Reread to be sure
  ConfigurationHandler cfHdl_re(file_path);
  std::vector<CameraConfiguration> cams_config_re =
      cfHdl_re.ParseConfiguration();
  ASSERT_GE(cams_config_re.size(), 1);
  ASSERT_EQ(cams_config_re[0].GetName().compare("Front"), 0);
  ASSERT_EQ(cams_config_re[0].GetBoolean("conf1"), true);
  ASSERT_EQ(cams_config_re[0].GetFloat("conf2"), 1.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
