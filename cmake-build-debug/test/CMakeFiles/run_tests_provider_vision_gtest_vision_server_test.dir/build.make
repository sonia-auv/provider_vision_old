# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/kevin/Documents/clion-2016.3.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/kevin/Documents/clion-2016.3.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug

# Utility rule file for run_tests_provider_vision_gtest_vision_server_test.

# Include the progress variables for this target.
include test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/progress.make

test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test:
	cd /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test && ../catkin_generated/env_cached.sh /usr/bin/python /opt/ros/kinetic/share/catkin/cmake/test/run_tests.py /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test_results/provider_vision/gtest-vision_server_test.xml /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/devel/lib/provider_vision/vision_server_test\ --gtest_output=xml:/home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test_results/provider_vision/gtest-vision_server_test.xml

run_tests_provider_vision_gtest_vision_server_test: test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test
run_tests_provider_vision_gtest_vision_server_test: test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/build.make

.PHONY : run_tests_provider_vision_gtest_vision_server_test

# Rule to build all files generated by this target.
test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/build: run_tests_provider_vision_gtest_vision_server_test

.PHONY : test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/build

test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/clean:
	cd /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/clean

test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/depend:
	cd /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/test /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test /home/kevin/Workspaces/ros_sonia_ws/src/provider_vision/cmake-build-debug/test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/run_tests_provider_vision_gtest_vision_server_test.dir/depend

