 FROM osrf/ros:noetic-desktop-full

 
# # Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update 
RUN apt-get install -y --no-install-recommends python3-rosdep

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install turtlebot stuff
RUN apt-get -y install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
  ros-noetic-turtlebot3*

RUN echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc


# install git
RUN apt-get update && apt-get install -y git


# # Build the Catkin workspace and ensure it's sourced
# RUN source /opt/ros/noetic/setup.bash \
#  && cd turtlebot3_ws \
#  && catkin_make
# RUN echo "source /turtlebot3_ws/devel/setup.bash" >> ~/.bashrc


# Set the working folder at startup

WORKDIR /catkin_ws

# Change the default shell to Bash

SHELL [ "/bin/bash" , "-c" ]

