#!/bin/sh

# Install the prerequisites for the ROS By Example code, Volume 1

sudo apt-get install ros-kinetic-turtlebot-bringup \
ros-kinetic-openni-* ros-kinetic-openni2-* \
ros-kinetic-freenect-* ros-kinetic-usb-cam \
ros-kinetic-audio-common gstreamer1.0-pocketsphinx \
ros-kinetic-slam-gmapping ros-kinetic-laser-* \
ros-kinetic-joystick-drivers python-rosinstall \
ros-kinetic-orocos-kdl ros-kinetic-python-orocos-kdl \
python-setuptools ros-kinetic-dynamixel-motor-* \
libopencv-dev python-opencv ros-kinetic-vision-opencv \
ros-kinetic-depthimage-to-laserscan ros-kinetic-arbotix \
ros-kinetic-turtlebot-teleop ros-kinetic-navigation \
git mercurial

