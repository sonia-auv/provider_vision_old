[![S.O.N.I.A. Logo](http://sonia.etsmtl.ca/wp-content/uploads/logo.jpg)](http://sonia.etsmtl.ca/en/)

:zap: *A software developed by S.O.N.I.A. team from ETS Montreal* :zap:

# Provider Vision Package

## Prerequisites
This software is developed using ROS and some of its dependencies.
It has been tested on a Ubuntu 14.04 OS with ROS indigo distribution.
The provider vision package also depends on other SONIA packages such as lib_atlas and lib_vision_sample.
In order to build and run this software, you will need these packages:
- Ubuntu 14.04
- ROS Indigo
- OpenCV 2.4.10
- lib_atlas
- lib_vision_sample

## Installation

### Installing ROS

```
    export ROS_DISTRO=indigo
    export ROS_DESKTOP=`lsb_release -cs`
    export PYTHONPATH='$PYTHONPATH:/opt/ros/indigo/lib/python2.7/dist-packages:/usr/lib/python2.7/dist-packages/'
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update -qq
    sudo apt-get install -qq -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-ros # Install ROS
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init # Setup rosdep
    rosdep update
    source /opt/ros/$ROS_DISTRO/setup.bash # Sourcing ROS exec
```

### Creating the Workspace

```
    mkdir -p ~/catkin_ws/src # Create workspace.
    cd ~/catkin_ws/src
    git clone https://github.com/sonia-auv/lib_atlas.git
    git clone https://github.com/sonia-auv/lib_vision_sample.git
    git clone https://github.com/sonia-auv/provider_vision.git
    catkin_init_workspace
    cd ~/catkin_ws/
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y # Install dependencies
    catkin_make # Build
```

### Launch

For running the vision provider:

`rosrun provider_vision provider_vision_node`

You can also run the unit test with this command at the root of your workspace

`catkin_make run_tests`

Finally, if you installed the `rqt_vision` package from S.O.N.I.A. Software distribution, you can run it by typing:

`rosrun rqt_vision rqt_vision_node`

## Directory Structure

???

## Documentation

???

## Contribute

???

## License

???
