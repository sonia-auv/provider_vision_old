# Change Log
All notable changes to this project will be documented in this file.

## [Unreleased]
### Added
- Adding some unit test for common features
- The vision server now support configuration of camera on the fly.

### Changed
- Change error handling in the whole code base to throw exceptions
- Move all ROS services and messages managment to the VisionServer class
- Changing the system to use context for managing cameras, filterchains and 
  detection tasks.
- 

## 1.1 - 2015-10-02
### Changed
- Removing some ROS otherhead
- Changing the logging system to ROS logging.
- Getting rid of vitals, integrating lib_atlas
- Changing the environment variable to ROS_SONIA_WS

## 1.0 - 2015-08-13
### Added

- Adding unit tests implemented with gtest
- Contour and ContourList for managing contours objects
- Adding bilateral filter for blurring image

### Changed
- Changing syntax of method to use commonly used syntax
- Changing in range filter to use LUV color space

### Fixed
- Removing FANN dependencies

## 1.0 - 2015-08-13
### Added

### Changed
- Removing vitals dependancies and linking the library to lib_atlas
- Integrating lib_vision to ROS as a dynamic library
