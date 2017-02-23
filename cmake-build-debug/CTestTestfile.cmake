# CMake generated Testfile for 
# Source directory: /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision
# Build directory: /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_provider_vision_roslaunch-check_launch "/home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test_results/provider_vision/roslaunch-check_launch.xml" "--return-code" "/home/kevin/Documents/clion-2016.3.3/bin/cmake/bin/cmake -E make_directory /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test_results/provider_vision" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test_results/provider_vision/roslaunch-check_launch.xml' '/home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/launch' ")
subdirs(gtest)
subdirs(test)
