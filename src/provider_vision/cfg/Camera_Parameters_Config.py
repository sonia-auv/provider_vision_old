## *********************************************************
## 
## File autogenerated for the provider_vision package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'BOTTOM_GIGE', 'lower': 'bottom_gige', 'srcline': 107, 'name': 'bottom_gige', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::BOTTOM_GIGE', 'field': 'DEFAULT::bottom_gige', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 9, 'description': 'True = Manual, False = Auto ', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_gain_mode', 'edit_method': '', 'default': True, 'level': 101, 'min': False, 'type': 'bool'}, {'srcline': 10, 'description': 'Value for the gain', 'max': 20.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_gain_value', 'edit_method': '', 'default': 1.0, 'level': 102, 'min': 0.0, 'type': 'double'}, {'srcline': 11, 'description': 'Value for the gamma', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_gamma_value', 'edit_method': '', 'default': 0.0, 'level': 103, 'min': -inf, 'type': 'double'}, {'srcline': 12, 'description': 'True = Manual, False = Auto', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_exposure_mode', 'edit_method': '', 'default': False, 'level': 104, 'min': False, 'type': 'bool'}, {'srcline': 13, 'description': 'Value for the exposure', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_exposure_value', 'edit_method': '', 'default': 1000.0, 'level': 105, 'min': -inf, 'type': 'double'}, {'srcline': 14, 'description': 'Value for the saturation', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_saturation_value', 'edit_method': '', 'default': 0.0, 'level': 106, 'min': -inf, 'type': 'double'}, {'srcline': 15, 'description': 'True = Manual, False = Auto ', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_shutter_mode', 'edit_method': '', 'default': False, 'level': 107, 'min': False, 'type': 'bool'}, {'srcline': 16, 'description': 'Value for the shutter', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_shutter_value', 'edit_method': '', 'default': 500.0, 'level': 108, 'min': -inf, 'type': 'double'}, {'srcline': 17, 'description': 'Value for the framerate', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_framerate_value', 'edit_method': '', 'default': 15.0, 'level': 109, 'min': -inf, 'type': 'double'}, {'srcline': 18, 'description': 'True = Manual , False = Auto', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_whitebalance_mode', 'edit_method': '', 'default': True, 'level': 111, 'min': False, 'type': 'bool'}, {'srcline': 19, 'description': 'Value for the red in the white balance', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_whitebalance_red_value', 'edit_method': '', 'default': 0.0, 'level': 112, 'min': -inf, 'type': 'double'}, {'srcline': 20, 'description': 'Value for the blue in the white balance', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_whitebalance_blue_value', 'edit_method': '', 'default': 0.0, 'level': 113, 'min': -inf, 'type': 'double'}, {'srcline': 21, 'description': 'Value for the auto brightness', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_autobrightness_mode', 'edit_method': '', 'default': True, 'level': 114, 'min': False, 'type': 'bool'}, {'srcline': 22, 'description': 'Value for the target brightness', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_autobrightness_target', 'edit_method': '', 'default': 100, 'level': 115, 'min': -2147483648, 'type': 'int'}, {'srcline': 23, 'description': 'Value for the variation of the brightness', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_autobrightness_variation', 'edit_method': '', 'default': 16, 'level': 116, 'min': -2147483648, 'type': 'int'}, {'srcline': 24, 'description': 'Value for the width on the image', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_width_value', 'edit_method': '', 'default': 968, 'level': 117, 'min': -2147483648, 'type': 'int'}, {'srcline': 25, 'description': 'Value for the height', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_height_value', 'edit_method': '', 'default': 608, 'level': 118, 'min': -2147483648, 'type': 'int'}, {'srcline': 26, 'description': 'Offset on the X axis', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_x_offset', 'edit_method': '', 'default': 484, 'level': 119, 'min': -2147483648, 'type': 'int'}, {'srcline': 27, 'description': 'Offset on the Y axis', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_y_offset', 'edit_method': '', 'default': 304, 'level': 120, 'min': -2147483648, 'type': 'int'}, {'srcline': 28, 'description': 'Value for the variation of the brightness', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'bottom_gige_format', 'edit_method': '', 'default': 17301513, 'level': 121, 'min': -2147483648, 'type': 'int'}], 'type': '', 'id': 1}, {'upper': 'FRONT_GIGE', 'lower': 'front_gige', 'srcline': 107, 'name': 'front_gige', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::FRONT_GIGE', 'field': 'DEFAULT::front_gige', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 32, 'description': 'True = Manual, False = Auto ', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_gain_mode', 'edit_method': '', 'default': True, 'level': 201, 'min': False, 'type': 'bool'}, {'srcline': 33, 'description': 'Value for the gain', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_gain_value', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 34, 'description': 'Value for the gamma', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_gamma_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 35, 'description': 'True = Manual, False = Auto', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_exposure_mode', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 36, 'description': 'Value for the exposure', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_exposure_value', 'edit_method': '', 'default': 1000.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 37, 'description': 'Value for the saturation', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_saturation_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 38, 'description': 'True = Manual, False = Auto ', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_shutter_mode', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 39, 'description': 'Value for the shutter', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_shutter_value', 'edit_method': '', 'default': 500.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 40, 'description': 'Value for the framerate', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_framerate_value', 'edit_method': '', 'default': 15.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 41, 'description': 'True = Manual , False = Auto', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_whitebalance_mode', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 42, 'description': 'Value for the red in the white balance', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_whitebalance_red_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 43, 'description': 'Value for the blue in the white balance', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_whitebalance_blue_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 44, 'description': 'Value for the auto brightness', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_autobrightness_mode', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 45, 'description': 'Value for the target brightness', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_autobrightness_target', 'edit_method': '', 'default': 100, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 46, 'description': 'Value for the variation of the brightness', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_autobrightness_variation', 'edit_method': '', 'default': 16, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 47, 'description': 'Value for the width on the image', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_width_value', 'edit_method': '', 'default': 968, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 48, 'description': 'Value for the height', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_height_value', 'edit_method': '', 'default': 608, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 49, 'description': 'Offset on the X axis', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_x_offset', 'edit_method': '', 'default': 484, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 50, 'description': 'Offset on the Y axis', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_y_offset', 'edit_method': '', 'default': 304, 'level': 0, 'min': -2147483648, 'type': 'int'}, {'srcline': 51, 'description': 'Value for the variation of the brightness', 'max': 2147483647, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_gige_format', 'edit_method': '', 'default': 17301513, 'level': 0, 'min': -2147483648, 'type': 'int'}], 'type': '', 'id': 2}, {'upper': 'FRONT_GUPPY', 'lower': 'front_guppy', 'srcline': 107, 'name': 'front_guppy', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::FRONT_GUPPY', 'field': 'DEFAULT::front_guppy', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 56, 'description': 'True = Manual, False = Auto ', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_gain_mode', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 57, 'description': 'Value for the gain', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_gain_value', 'edit_method': '', 'default': 1.0, 'level': 302, 'min': 680.0, 'type': 'double'}, {'srcline': 58, 'description': 'Value for the gamma', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_gamma_value', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 59, 'description': 'True = Manual, False = Auto', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_exposure_mode', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 60, 'description': 'Value for the exposure', 'max': 205.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_exposure_value', 'edit_method': '', 'default': 100.0, 'level': 0, 'min': 50.0, 'type': 'double'}, {'srcline': 61, 'description': 'Value for the saturation', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_saturation_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 62, 'description': 'True = Manual, False = Auto ', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_shutter_mode', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 63, 'description': 'Value for the shutter', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_shutter_value', 'edit_method': '', 'default': 500.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 64, 'description': 'Value for the framerate', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_framerate_value', 'edit_method': '', 'default': 15.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 65, 'description': 'True = Manual , False = Auto', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_whitebalance_mode', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 66, 'description': 'Value for the red in the white balance', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_whitebalance_red_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}, {'srcline': 67, 'description': 'Value for the blue in the white balance', 'max': inf, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/piranha/Workspaces/ros_sonia_ws/src/provider_vision/cfg/camera_dynamic_reconf.cfg', 'name': 'front_guppy_whitebalance_blue_value', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -inf, 'type': 'double'}], 'type': '', 'id': 3}], 'parameters': [], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

